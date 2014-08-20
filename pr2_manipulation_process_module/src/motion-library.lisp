;;; Copyright (c) 2012, Jan Winkler <winkler@informatik.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :pr2-manipulation-process-module)

(define-hook on-execute-grasp-with-effort (object-name))
(define-hook on-execute-grasp-gripper-closed
    (object-name gripper-effort gripper-close-pos side pregrasp-pose safe-pose))
(define-hook on-execute-grasp-gripper-positioned-for-grasp
    (object-name gripper-effort gripper-close-pos side pregrasp-pose safe-pose))
(define-hook on-execute-grasp-pregrasp-reached
    (object-name gripper-effort gripper-close-pos side pregrasp-pose safe-pose))

(defun execute-grasp (&key (mode :traditional);smooth)
                        object-name
                        object-pose
                        pregrasp-pose
                        grasp-pose
                        side (gripper-open-pos 0.2)
                        (gripper-close-pos 0.0)
                        allowed-collision-objects
                        safe-pose
                        (gripper-effort 100.0))
  (let ((allowed-collision-objects (append
                                    allowed-collision-objects
                                    (list object-name))))
    (cond ((eql mode :traditional)
           (execute-grasp-traditional
            :object-name object-name
            :object-pose object-pose
            :pregrasp-pose pregrasp-pose
            :grasp-pose grasp-pose
            :side side
            :gripper-open-pos gripper-open-pos
            :gripper-close-pos gripper-close-pos
            :allowed-collision-objects allowed-collision-objects
            :safe-pose safe-pose
            :gripper-effort gripper-effort))
          ((eql mode :smooth)
           (execute-grasp-smooth
            :object-name object-name
            :object-pose object-pose
            :pregrasp-pose pregrasp-pose
            :grasp-pose grasp-pose
            :side side
            :gripper-open-pos gripper-open-pos
            :gripper-close-pos gripper-close-pos
            :allowed-collision-objects allowed-collision-objects
            :safe-pose safe-pose
            :gripper-effort gripper-effort))
          (t (ros-error (pr2 grasp) "Invalid grasping motion executer: ~a" mode)))))

(defun execute-grasp-smooth (&key object-name
                               object-pose
                               pregrasp-pose
                               grasp-pose
                               side (gripper-open-pos 0.2)
                               (gripper-close-pos 0.0)
                               allowed-collision-objects
                               safe-pose
                               (gripper-effort 100.0))
  (declare (ignorable object-pose gripper-close-pos))
  ;; Generate trajectories for grasp and pregrasp
  (let* ((pregrasp-trajectory
           (execute-move-arm-pose
            side pregrasp-pose
            :allowed-collision-objects allowed-collision-objects
            :plan-only t))
         (pregrasp-end-state (robot-trajectory-end-joint-state pregrasp-trajectory)))
    (unless pregrasp-trajectory
      (ros-warn (pr2 grasp) "Unable to assume pregrasp pose.")
      (cpl:fail 'manipulation-pose-unreachable))
    (let ((grasp-trajectory
            (execute-move-arm-pose
             side grasp-pose
             :allowed-collision-objects allowed-collision-objects
             :plan-only t
             :start-state (roslisp:make-message
                           "moveit_msgs/RobotState"
                           :joint_state pregrasp-end-state))))
      (unless grasp-trajectory
        (ros-warn (pr2 grasp) "Unable to assume grasp pose.")
        (cpl:fail 'manipulation-pose-unreachable))
      (let ((merged-trajectory (merge-robot-trajectories pregrasp-trajectory grasp-trajectory)))
        (ros-info (pr2 grasp) "Opening gripper")
        (open-gripper side :max-effort gripper-effort :position gripper-open-pos)
        (moveit:remove-collision-object object-name)
        (unwind-protect
             (moveit::execute-trajectory merged-trajectory)
          (moveit:add-collision-object object-name))
        (when (< (get-gripper-state side) 0.01)
          (ros-warn
           (pr2 grasp)
           "Missed the object. Going into fallback pose for side ~a." side)
          (open-gripper side)
          (execute-move-arm-pose side pregrasp-pose
                                 :allowed-collision-objects
                                 allowed-collision-objects)
          (moveit:add-collision-object object-name)
          (when safe-pose
            (execute-move-arm-pose side safe-pose
                                   :allowed-collision-objects
                                   allowed-collision-objects))
          (cpl:fail 'cram-plan-failures:object-lost))))
    (moveit:add-collision-object object-name)
    (let ((link-frame
            (cut:var-value
             '?link
             (first
              (crs:prolog
               `(manipulator-link ,side ?link))))))
      (moveit:attach-collision-object-to-link
       object-name link-frame))))

(defun merge-robot-trajectories (&rest trajectories)
  (when trajectories
    (let ((first-trajectory (first trajectories))
          (trajectory-gap-time 0.1))
      (roslisp:with-fields (joint_trajectory) first-trajectory
        (roslisp:with-fields (header joint_names) joint_trajectory
          (roslisp:make-message
           "moveit_msgs/RobotTrajectory"
           :joint_trajectory
           (roslisp:make-message
            "trajectory_msgs/JointTrajectory"
            :header header
            :joint_names joint_names
            :points
            (map 'vector #'identity
                 (let ((traj-end-time 0.0))
                   (loop for trajectory in trajectories
                         appending (prog1
                                       (map 'list
                                            (lambda (point-old)
                                              (roslisp:with-fields (positions
                                                                    velocities
                                                                    accelerations
                                                                    time_from_start) point-old
                                                (roslisp:make-message
                                                 "trajectory_msgs/JointTrajectoryPoint"
                                                 :positions positions
                                                 :velocities velocities
                                                 :accelerations accelerations
                                                 :time_from_start (+ time_from_start traj-end-time))))
                                            (roslisp:with-fields (joint_trajectory) trajectory
                                              (roslisp:with-fields (points) joint_trajectory
                                                points)))
                                     (roslisp:with-fields (joint_trajectory) trajectory
                                       (roslisp:with-fields (points) joint_trajectory
                                         (let ((last-pt (elt points (1- (length points)))))
                                           (roslisp:with-fields (time_from_start) last-pt
                                             (setf traj-end-time (+ time_from_start
                                                                    trajectory-gap-time)))))))
                           into aggr-points
                         finally (return aggr-points)))))))))))

(defun robot-trajectory-end-joint-state (robot-trajectory)
  (when robot-trajectory
    (roslisp:with-fields (joint_trajectory) robot-trajectory
      (roslisp:with-fields (header joint_names points) joint_trajectory
        (when points
          (joint-trajectory-point->joint-state
           header joint_names (elt points (1- (length points)))))))))

(defun joint-trajectory-point->joint-state (header joint-names trajectory-point)
  (roslisp:with-fields (positions) trajectory-point
    (roslisp:make-message
     "sensor_msgs/JointState"
     :header header
     :name joint-names
     :position positions)))

(defun execute-grasp-traditional (&key object-name
                                    object-pose
                                    pregrasp-pose
                                    grasp-pose
                                    side (gripper-open-pos 0.2)
                                    (gripper-close-pos 0.0)
                                    allowed-collision-objects
                                    safe-pose
                                    (gripper-effort 100.0))
  (declare (ignore object-pose))
  ;; Gather hook'ed information for the grasp
  (let ((gripper-effort (or (first (on-execute-grasp-with-effort
                                    object-name))
                            gripper-effort)))
    (let ((link-frame
            (cut:var-value
             '?link
             (first
              (crs:prolog
               `(manipulator-link ,side ?link))))))
      (ros-info (pr2 grasp) "Executing pregrasp for side ~a~%" side)
      (cpl:with-retry-counters ((pregrasp-retry 1))
        (cpl:with-failure-handling
            ((cram-plan-failures::manipulation-failure (f)
               (declare (ignore f))
               (ros-error (pr2 grasp)
                          "Failed to go into pregrasp pose for side ~a."
                          side)
               (cpl:do-retry pregrasp-retry
                 (execute-move-arm-pose side safe-pose)
                 (cpl:retry))
               (when safe-pose
                 (cpl:fail 'manipulation-pose-unreachable))))
          (execute-move-arm-pose side pregrasp-pose)))
      (ros-info (pr2 grasp) "Opening gripper")
      (open-gripper side :max-effort gripper-effort :position gripper-open-pos)
      (unless object-name
        (ros-warn (pr2 grasp) "You didn't specify an object name to grasp. This might cause the grasping to fail because of a misleading collision environment configuration."))
      (when object-name (moveit:remove-collision-object object-name))
      (on-execute-grasp-pregrasp-reached
       object-name gripper-effort gripper-close-pos side pregrasp-pose safe-pose)
      (ros-info (pr2 grasp) "Executing grasp for side ~a~%" side)
      (cpl:with-failure-handling
          ((manipulation-failed (f)
             (declare (ignore f))
             (ros-error
              (pr2 grasp)
              "Failed to go into grasp pose for side ~a."
              side)
             (when object-name (moveit:add-collision-object object-name))
             (cpl:fail 'manipulation-pose-unreachable)))
        (execute-move-arm-pose side grasp-pose :ignore-collisions t))
      (on-execute-grasp-gripper-positioned-for-grasp
       object-name gripper-effort gripper-close-pos side pregrasp-pose safe-pose)
      (ros-info (pr2 grasp) "Closing gripper")
      (close-gripper side :max-effort gripper-effort
                          :position gripper-close-pos)
      (on-execute-grasp-gripper-closed
       object-name gripper-effort gripper-close-pos side pregrasp-pose safe-pose)
      (when (< (get-gripper-state side) 0.01);;gripper-close-pos)
        (cpl:with-failure-handling
            ((manipulation-failed (f)
               (declare (ignore f))
               (ros-error
                (pr2 grasp)
                "Failed to go into fallback pose for side ~a. Retrying."
                side)
               (cpl:retry))
             (moveit:pose-not-transformable-into-link (f)
               (declare (ignore f))
               (ros-warn
                (pr2 grasp)
                "TF error. Retrying.")
               (cpl:retry)))
          (ros-warn
           (pr2 grasp)
           "Missed the object. Going into fallback pose for side ~a." side)
          (open-gripper side)
          (execute-move-arm-pose side pregrasp-pose
                                 :allowed-collision-objects
                                 allowed-collision-objects)
          (moveit:add-collision-object object-name)
          (when safe-pose
            (execute-move-arm-pose side safe-pose
                                   :allowed-collision-objects
                                   allowed-collision-objects))
          (cpl:fail 'cram-plan-failures:object-lost)))
      (when object-name
        (moveit:add-collision-object object-name)
        (moveit:attach-collision-object-to-link
         object-name link-frame)))))

(defun execute-putdown (&key object-name
                          pre-putdown-pose putdown-pose
                          unhand-pose side
                          (gripper-open-pos 0.2)
                          allowed-collision-objects)
  (let ((allowed-collision-objects (append
                                    allowed-collision-objects
                                    (list object-name))))
    (ros-info
     (pr2 putdown) "Executing pre-putdown for side ~a~%" side)
    (publish-pose pre-putdown-pose "/preputdownpose")
    (cpl:with-failure-handling
        ((manipulation-pose-unreachable (f)
           (declare (ignore f))
           (ros-error
            (pr2 putdown) "Failed to go into preputdown pose for side ~a." side)
           (cpl:fail 'cram-plan-failures:manipulation-failed))
         (manipulation-failed (f)
           (declare (ignore f))
           (ros-error
            (pr2 putdown) "Failed to go into preputdown pose for side ~a." side))
         (moveit:pose-not-transformable-into-link (f)
           (declare (ignore f))
           (cpl:retry)))
      (execute-move-arm-pose
       side pre-putdown-pose
       :allowed-collision-objects allowed-collision-objects))
    (ros-info (pr2 putdown) "Executing putdown for side ~a~%" side)
    (cpl:with-failure-handling
        ((manipulation-pose-unreachable (f)
           (declare (ignore f))
           (ros-error (pr2 putdown)
                              "Failed to go into putdown pose for side ~a."
                              side))
         (manipulation-failed (f)
           (declare (ignore f))
           (ros-error (pr2 putdown)
                              "Failed to go into putdown pose for side ~a."
                              side))
         (moveit:pose-not-transformable-into-link (f)
           (declare (ignore f))
           (cpl:retry)))
      (execute-move-arm-pose side putdown-pose
       :allowed-collision-objects allowed-collision-objects
       :collidable-objects `(,object-name)))
    (ros-info (pr2 putdown) "Opening gripper")
    (open-gripper side :max-effort 50.0 :position gripper-open-pos)
    (moveit:detach-collision-object-from-link
     object-name (cut:var-value
                  '?link
                  (first
                   (crs:prolog
                    `(manipulator-link ,side ?link)))))
    (ros-info (pr2 putdown) "Executing unhand for side ~a~%" side)
    (let ((ignore-collisions nil))
      (cpl:with-failure-handling
          ((manipulation-failure (f)
             (declare (ignore f))
             (ros-error
              (pr2 putdown)
              "Failed to go into unhand pose for side ~a. Retrying while ignoring collisions." side)
             (setf ignore-collisions t)
             (cpl:retry))
           (moveit:pose-not-transformable-into-link (f)
             (declare (ignore f))
             (cpl:retry)))
        (execute-move-arm-pose
         side unhand-pose
         :allowed-collision-objects (unless ignore-collisions
                                      allowed-collision-objects)
         :ignore-collisions ignore-collisions)))
    (ros-info (pr2 manip-pm) "Putdown complete.")))

(defun lift-grasped-object-with-one-arm (side distance)
  "Executes a lifting motion with the `side' arm which grasped the
object in order to lift it at `distance' form the supporting plane"
  (let* ((frame-id (cut:var-value
                    '?link
                    (first
                     (crs:prolog
                      `(manipulator-link ,side ?link)))))
         (raised-arm-pose
           (moveit:ensure-pose-stamped-transformed
            (tf:make-pose-stamped frame-id (ros-time)
                                  (tf:make-3d-vector 0 0 distance)
                                  (tf:make-identity-rotation))
            "/torso_lift_link" :ros-time t)))
    (unless raised-arm-pose
      (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
    (execute-move-arm-pose side raised-arm-pose :ignore-collisions t)))

(defun relative-grasp-pose (pose pose-offset)
  (let* ((stamp (ros-time))
         (target-frame "/torso_lift_link"))
    ;; NOTE(winkler): Right now, we check whether a transformation can
    ;; actually be done at the current time. This has to be checked
    ;; because sometimes the upcoming wait-for-transform waits forever
    ;; (for yet unknown reasons). This is a preliminary fix, though.
    (let ((pose-offsetted (tf:pose->pose-stamped
                           (tf:frame-id pose)
                           stamp
                           (cl-transforms:transform-pose
                            (cl-transforms:pose->transform pose)
                            pose-offset))))
      (moveit:ensure-pose-stamped-transformed
       pose-offsetted target-frame :ros-time t))))
