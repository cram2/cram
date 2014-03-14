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

(defparameter *carry-pose-right* (tf:make-pose-stamped
                                  "/base_footprint" 0.0
                                  (cl-transforms:make-3d-vector
                                   0.2 -0.55 1.30)
                                  (cl-transforms:euler->quaternion :ay (/ pi 2))))
(defparameter *carry-pose-left* (tf:make-pose-stamped
                                 "/base_footprint" 0.0
                                 (cl-transforms:make-3d-vector
                                  0.2 0.55 1.30)
                                 (cl-transforms:euler->quaternion :ay (/ pi 2))))

(defparameter *top-grasp* (cl-transforms:euler->quaternion :ay (/ pi 2)))
(defparameter *front-grasp* (cl-transforms:make-identity-rotation))
(defparameter *left-grasp* (cl-transforms:euler->quaternion :az (/ pi -2)))
(defparameter *right-grasp* (cl-transforms:euler->quaternion :az (/ pi 2)))

(defparameter *pot-relative-right-handle-transform*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector
    0.135 -0.012 0.08)
   (cl-transforms:make-quaternion
    -0.17903158312000483d0 -0.6841142429564597d0
    0.6850543286185364d0 -0.17503131625700874d0)))

(defparameter *pot-relative-left-handle-transform*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector
    -0.135 -0.008 0.08)
   (cl-transforms:make-quaternion
    -0.6770480569314481d0 0.11600823330627819d0
    0.16801192666809586d0 0.7070502180947129d0)))

(defparameter *grasp-approach-distance* 0.10
  "Distance to approach the object. This parameter is used to
  calculate the pose to approach with move_arm.")
(defparameter *grasp-distance* 0.18
  "Tool length to calculate the pre-grasp pose, i.e. the pose at which
  the gripper is closed.")
(defparameter *pre-put-down-distance* 0.07
  "Distance above the goal before putting down the object")

(defparameter *max-graspable-size*
  (cl-transforms:make-3d-vector 0.15 0.15 0.30))

(defun execute-grasp (&key object-name
                        object-pose
                        pregrasp-pose
                        grasp-pose
                        side (gripper-open-pos 0.2)
                        (gripper-close-pos 0.0)
                        allowed-collision-objects
                        safe-pose
                        (gripper-effort 25.0))
  (declare (ignore object-pose))
  (let ((link-frame (ecase side
                      (:left "l_wrist_roll_link")
                      (:right "r_wrist_roll_link")))
        (allowed-collision-objects (append
                                    allowed-collision-objects
                                    (list object-name))))
    (ros-info (pr2 grasp) "Executing pregrasp for side ~a~%" side)
    (cpl:with-failure-handling
        ((manipulation-failed (f)
           (declare (ignore f))
           (ros-error (pr2 grasp)
                              "Failed to go into pregrasp pose for side ~a."
                              side)
           (when safe-pose
             (execute-move-arm-pose side safe-pose))
           (cpl:fail 'manipulation-pose-unreachable))
         (manipulation-pose-unreachable (f)
           (declare (ignore f))
           (ros-error (pr2 grasp)
                              "Failed to go into pregrasp pose for side ~a."
                              side)
           (when safe-pose
             (execute-move-arm-pose side safe-pose)))
         (moveit:pose-not-transformable-into-link (f)
           (declare (ignore f))
           (cpl:retry)))
      (execute-move-arm-pose side pregrasp-pose))
       ;; :allowed-collision-objects allowed-collision-objects))
       ;; NOTE(winkler): Removed the allowed collision objects from
       ;; the pregrasp motion call for now.
    (ros-info (pr2 grasp) "Opening gripper")
    (open-gripper side :max-effort gripper-effort :position gripper-open-pos)
    (unless object-name
      (ros-warn (pr2 grasp) "You didn't specify an object name to
    grasp. This might cause the grasping to fail because of a
    misleading collision environment configuration."))
    (when object-name (moveit:remove-collision-object object-name))
    (ros-info (pr2 grasp) "Executing grasp for side ~a~%" side)
    (cpl:with-failure-handling
        ((manipulation-failed (f)
           (declare (ignore f))
           (ros-error (pr2 grasp)
                              "Failed to go into grasp pose for side ~a."
                              side)
           (when object-name
             (moveit:add-collision-object object-name))
	   (cpl:fail 'manipulation-pose-unreachable))
         (moveit::pose-not-transformable-into-link (f)
           (declare (ignore f))
           (cpl:retry)))
      (execute-move-arm-pose side grasp-pose))
       ;; :allowed-collision-objects allowed-collision-objects))
       ;; NOTE(winkler): See above. The PR2 was throwing the object
       ;; off the table when going from pregrasp to grasp pose.
    (ros-info (pr2 grasp) "Closing gripper")
    (close-gripper side :max-effort gripper-effort :position gripper-close-pos)
    (sleep 2)
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
       object-name link-frame))))

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
       :ignore-collisions t))
    (ros-info (pr2 putdown) "Opening gripper")
    (open-gripper side :max-effort 50.0 :position gripper-open-pos)
    (moveit:detach-collision-object-from-link
     object-name (ecase side
                   (:left "l_wrist_roll_link")
                   (:right "r_wrist_roll_link")))
    (ros-info (pr2 putdown) "Executing unhand for side ~a~%" side)
    (cpl:with-failure-handling
        ((manipulation-pose-unreachable (f)
           (declare (ignore f))
           (ros-error
            (pr2 putdown)
            "Failed to go into unhand pose for side ~a. Retrying." side)
           (cpl:retry))
         (manipulation-failed (f)
           (declare (ignore f))
           (ros-error
            (pr2 putdown)
            "Failed to go into unhand pose for side ~a. Retrying." side)
           (cpl:retry))
         (moveit:pose-not-transformable-into-link (f)
           (declare (ignore f))
           (cpl:retry)))
      (execute-move-arm-pose
       side unhand-pose
       :allowed-collision-objects allowed-collision-objects
       :ignore-collisions t)
      (ros-info (pr2 manip-pm) "Putdown complete."))))

(defun get-lifting-grasped-object-arm-pose (side distance)
  "Returns the lifting pose for the `side' robot arm in order to
lift the grasped object at the `distance' from the supporting plane."
  (let* ((wrist-transform (tf:lookup-transform
                           *tf*
                           :time 0
                           :source-frame (ecase side
                                           (:right "r_wrist_roll_link")
                                           (:left "l_wrist_roll_link"))
                           :target-frame "/torso_lift_link"))
         (lifted-pose (tf:make-pose-stamped
                       (tf:frame-id wrist-transform)
                       (tf:stamp wrist-transform)
                       (cl-transforms:v+
                        (cl-transforms:translation wrist-transform)
                        (cl-transforms:make-3d-vector 0 0 distance))
                       (cl-transforms:rotation wrist-transform))))
    lifted-pose))

(defun lift-grasped-object-with-both-arms (distance)
  "Executes a parallel lifting motion with both arms in order to lift
the object which is grasped with both arms at `distance' form the
supporting plane"
  (cpl-impl:par
    ;; NOTE(winkler): This is likely to only work good with one
    ;; arm. Two arms that need to be synched in parallel motion need
    ;; to take into account that they constraint their respective
    ;; motion, according to the object to be lifted. This means that
    ;; the function `get-lifting-grasped-object-arm-pose' needs to be
    ;; reworked (or duplicated - one for one arm, one for both arms).
    (execute-move-arm-pose :left (get-lifting-grasped-object-arm-pose
                                  :left distance))
    (execute-move-arm-pose :right (get-lifting-grasped-object-arm-pose
                                   :right distance))))

(defun lift-grasped-object-with-one-arm (side distance)
  "Executes a lifting motion with the `side' arm which grasped the
object in order to lift it at `distance' form the supporting plane"
  (let* ((frame-id (ecase side
                     (:right "r_wrist_roll_link")
                     (:left "l_wrist_roll_link")))
         (raised-arm-pose
           (moveit:ensure-pose-stamped-transformed
            (tf:make-pose-stamped frame-id (ros-time)
                                  (tf:make-3d-vector 0 0 distance)
                                  (tf:make-identity-rotation))
            "/torso_lift_link" :ros-time t)))
    (unless raised-arm-pose
      (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
    (execute-move-arm-pose side raised-arm-pose :ignore-collisions t)))

(defun open-drawer (pose side &optional (distance *grasp-distance*))
  "Generates and executes a pull trajectory for the `side' arm in order to open the
   drawer whose handle is at `pose'. The generated poses of the pull trajectory are
   relative to the drawer handle `pose' and additionally transformed into base_footprint
   frame. Finally the new pose of the drawer handle is returned."
  (cl-tf:wait-for-transform *tf*
                            :timeout 1.0
                            :time (tf:stamp pose)
                            :source-frame (tf:frame-id pose)
                            :target-frame "base_footprint")
  (let* ((pose-transform (cl-transforms:pose->transform pose))
         (pre-grasp-pose
           (cl-tf:transform-pose cram-roslisp-common:*tf*
                                 :pose (tf:pose->pose-stamped
                                        (tf:frame-id pose) (tf:stamp pose)
                                        (cl-transforms:transform-pose
                                         pose-transform
                                         (cl-transforms:make-pose
                                          (cl-transforms:make-3d-vector
                                           -0.25 0.0 0.0)
                                          (cl-transforms:make-identity-rotation))))
                                 :target-frame "base_footprint"))
         (grasp-pose
           (cl-tf:transform-pose cram-roslisp-common:*tf*
                                 :pose (tf:pose->pose-stamped
                                        (tf:frame-id pose) (tf:stamp pose)
                                        (cl-transforms:transform-pose
                                         pose-transform
                                         (cl-transforms:make-pose
                                          (cl-transforms:make-3d-vector
                                           -0.20 0.0 0.0)
                                          (cl-transforms:make-identity-rotation))))
                                 :target-frame "base_footprint"))
         (open-pose
           (cl-tf:transform-pose cram-roslisp-common:*tf*
                                 :pose (tf:pose->pose-stamped
                                        (tf:frame-id pose) (tf:stamp pose)
                                        (cl-transforms:transform-pose
                                         pose-transform
                                         (cl-transforms:make-pose
                                          (cl-transforms:make-3d-vector
                                           (- -0.20 distance) 0.0 0.0)
                                          (cl-transforms:make-identity-rotation))))
                                 :target-frame "base_footprint"))
         (arm-away-pose
           (cl-tf:transform-pose cram-roslisp-common:*tf*
                                 :pose (tf:pose->pose-stamped
                                        (tf:frame-id pose) (tf:stamp pose)
                                        (cl-transforms:transform-pose
                                         pose-transform
                                         (cl-transforms:make-pose
                                          (cl-transforms:make-3d-vector
                                           (+ -0.10 (- -0.20 distance)) 0.0 0.0)
                                          (cl-transforms:make-identity-rotation))))
                                 :target-frame "base_footprint"))
         (pre-grasp-ik (lazy-car (get-ik side pre-grasp-pose)))
         (grasp-ik (lazy-car (get-ik side grasp-pose)))
         (open-ik (lazy-car (get-ik side open-pose)))
         (arm-away-ik (lazy-car (get-ik side arm-away-pose))))
    (unless pre-grasp-ik
      (error 'manipulation-pose-unreachable
             :format-control "Pre-grasp pose for handle not reachable: ~a"
             :format-arguments (list pre-grasp-pose)))
    (unless grasp-ik
      (error 'manipulation-pose-unreachable
             :format-control "Grasp pose for handle not reachable: ~a"
             :format-arguments (list grasp-pose)))
    (unless open-ik
      (error 'manipulation-pose-unreachable
             :format-control "Open pose for handle not reachable."))
    (unless arm-away-ik
      (error 'manipulation-pose-unreachable
             :format-control "Arm-away pose after opening drawer is not reachable."))
    (open-gripper side)
    (execute-arm-trajectory side (ik->trajectory pre-grasp-ik))
    (execute-arm-trajectory side (ik->trajectory grasp-ik))
    (close-gripper side)
    (execute-arm-trajectory side (ik->trajectory open-ik))
    (open-gripper side)
    (execute-arm-trajectory side (ik->trajectory arm-away-ik))
    (tf:pose->pose-stamped
     (tf:frame-id pose) (tf:stamp pose)
     (cl-transforms:transform-pose
      pose-transform
      (cl-transforms:make-pose
       (cl-transforms:make-3d-vector
        (- distance) 0.0 0.0)
       (cl-transforms:make-identity-rotation))))))

(defun close-drawer (pose side &optional (distance *grasp-distance*))
  "Generates and executes a push trajectory for the `side' arm in order to close
   the drawer whose handle is at `pose'. The generated poses of the push trajectory
   are relative to the drawer handle `pose' and additionally transformed into
   base_footprint frame. Finally the new pose of the drawer handle is returned."
  (cl-tf:wait-for-transform *tf*
                            :timeout 1.0
                            :time (tf:stamp pose)
                            :source-frame (tf:frame-id pose)
                            :target-frame "base_footprint")
  (let* ((pose-transform (cl-transforms:pose->transform pose))
         (pre-grasp-pose
           (cl-tf:transform-pose *tf*
                                 :pose (tf:pose->pose-stamped
                                        (tf:frame-id pose) (tf:stamp pose)
                                        (cl-transforms:transform-pose
                                         pose-transform
                                         (cl-transforms:make-transform
                                          (cl-transforms:make-3d-vector
                                           -0.25 0.0 0.0)
                                          (cl-transforms:make-identity-rotation))))
                                 :target-frame "base_footprint"))
         (grasp-pose ;; (tool-goal-pose->wrist-goal-pose pose)
           (cl-tf:transform-pose *tf*
                                 :pose (tf:pose->pose-stamped
                                        (tf:frame-id pose) (tf:stamp pose)
                                        (cl-transforms:transform-pose
                                         pose-transform
                                         (cl-transforms:make-transform
                                          (cl-transforms:make-3d-vector
                                           -0.20 0.0 0.0)
                                          (cl-transforms:make-identity-rotation))))
                                 :target-frame "base_footprint"))
         (close-pose
           (cl-tf:transform-pose *tf*
                                 :pose (tf:pose->pose-stamped
                                        (tf:frame-id pose) (tf:stamp pose)
                                        (cl-transforms:transform-pose
                                         pose-transform
                                         (cl-transforms:make-transform
                                          (cl-transforms:make-3d-vector
                                           (+ -0.20 distance) 0.0 0.0)
                                          (cl-transforms:make-identity-rotation))))
                                 :target-frame "base_footprint"))
         (arm-away-pose
           (cl-tf:transform-pose *tf*
                                 :pose (tf:pose->pose-stamped
                                        (tf:frame-id pose) (tf:stamp pose)
                                        (cl-transforms:transform-pose
                                         pose-transform
                                         (cl-transforms:make-transform
                                          (cl-transforms:make-3d-vector
                                           -0.15 0.0 0.0)
                                          (cl-transforms:make-identity-rotation))))
                                 :target-frame "base_footprint"))
         (pre-grasp-ik (lazy-car (get-ik side pre-grasp-pose)))
         (grasp-ik (lazy-car (get-ik side grasp-pose)))
         (close-ik (lazy-car (get-ik side close-pose)))
         (arm-away-ik (lazy-car (get-ik side arm-away-pose))))
    (unless pre-grasp-ik
      (error 'manipulation-pose-unreachable
             :format-control "Pre-grasp pose for handle not reachable."))
    (unless grasp-ik
      (error 'manipulation-pose-unreachable
             :format-control "Grasp pose for handle not reachable."))
    (unless close-ik
      (error 'manipulation-pose-unreachable
             :format-control "Close pose for handle not reachable."))
    (unless arm-away-ik
      (error 'manipulation-pose-unreachable
             :format-control "Arm-away pose after closing the drawer is not reachable."))
    (open-gripper side)
    (execute-arm-trajectory side (ik->trajectory pre-grasp-ik))
    (execute-arm-trajectory side (ik->trajectory grasp-ik))
    (close-gripper side)
    (execute-arm-trajectory side (ik->trajectory close-ik))
    (open-gripper side)
    (execute-arm-trajectory side (ik->trajectory arm-away-ik))
    (tf:pose->pose-stamped
     (tf:frame-id pose) (tf:stamp pose)
     (cl-transforms:transform-pose
      pose-transform
      (cl-transforms:make-pose
       (cl-transforms:make-3d-vector
        (+ distance) 0.0 0.0)
       (cl-transforms:make-identity-rotation))))))

(defun check-valid-gripper-state (side &key (min-position 0.01) safety-ik safety-pose)
  (when (< (get-gripper-state side) min-position)
    (clear-collision-objects)
    (open-gripper side)
    (cond ((and safety-ik safety-pose)
           (ros-error
            (pr2 manip-pm)
            "Only one of `safety-ik' and `safety-pose' can be specified."))
          (safety-ik
           (execute-arm-trajectory side (ik->trajectory safety-ik)))
          (safety-pose
           (execute-move-arm-pose side safety-pose))
          (t
           (ros-error
            (pr2 manip-pm)
            "Neither `safety-ik' nor `safety-pose' specified. Not moving gripper to a safe pose.")))
    (cpl:fail 'object-lost)))

(defun park-grasped-object-with-one-arm (obj side &optional obstacles)
  (when obstacles
    (clear-collision-objects)
    (sem-map-coll-env:publish-semantic-map-collision-objects)
    (dolist (obstacle (cut:force-ll obstacles))
      (register-collision-object obstacle)))
  (let ((orientation (calculate-carry-orientation
                      obj side
                      (list *top-grasp* (cl-transforms:make-identity-rotation))))
        (carry-pose (ecase side
                      (:right *carry-pose-right*)
                      (:left *carry-pose-left*))))
    (if orientation
        (execute-move-arm-pose side (tf:copy-pose-stamped carry-pose :orientation orientation)
                               :allowed-collision-objects
                               (list (desig-prop-value obj 'desig-props:name)))
        (execute-move-arm-pose side carry-pose
                               :allowed-collision-objects
                               (list (desig-prop-value obj 'desig-props:name))))))

(defun park-grasped-object-with-two-arms (obj side &optional obstacles)
  "Moves the object which was grasped with two arms into a position in
front of the torso, keeping original gripper distance and
orientation. The center point between the two grippers is positioned
directly in front of `torso_lift_link'."
  (declare (ignore obj side))
  (when obstacles
    (clear-collision-objects)
    (dolist (obstacle (cut:force-ll obstacles))
      (register-collision-object obstacle)))
  (let* ((r-solution (get-fk-current-state :right
                                           :target-links
                                           (vector "r_wrist_roll_link")))
         (r-pose-stamped (cdr (first r-solution)))
         (l-solution (get-fk-current-state :left
                                           :target-links
                                           (vector "l_wrist_roll_link")))
         (l-pose-stamped (cdr (first l-solution)))
         (c-target-pose-stamped (tf:make-pose-stamped
                                 (tf:frame-id r-pose-stamped)
                                 (tf:stamp r-pose-stamped)
                                 (tf:make-3d-vector 0.5 0.0 0.0)
                                 (tf:make-identity-rotation)))
         ;; NOTE(winkler): This function assumes that all grasped
         ;; handles are on the same z-level. If they are not, the
         ;; object will be rotated until the two grippers are on the
         ;; same z-level. This could potentially cause problems while
         ;; putting it down, but at the moment we don't need to care
         ;; much about it. Once the z-level differs, the exact x- and
         ;; y-distance between the grippers with respect to correct
         ;; object-rotation needs to be taken into account. So, this
         ;; function is subject to changes in the future as necessary
         ;; and as problems arise.
         (dist (tf:v-dist (tf:origin r-pose-stamped)
                          (tf:origin l-pose-stamped)))
         (r-new-pose (tf:make-pose-stamped
                      (tf:frame-id r-pose-stamped)
                      (tf:stamp r-pose-stamped)
                      (tf:v+ (tf:origin c-target-pose-stamped)
                             (tf:make-3d-vector
                              0.0 (- (/ dist 2)) 0.0))
                      (tf:orientation r-pose-stamped)))
         (l-new-pose (tf:make-pose-stamped
                      (tf:frame-id l-pose-stamped)
                      (tf:stamp l-pose-stamped)
                      (tf:v+ (tf:origin c-target-pose-stamped)
                             (tf:make-3d-vector
                              0.0 (/ dist 2) 0.0))
                      (tf:orientation l-pose-stamped)))
         (l-ik (get-ik :left l-new-pose))
         (r-ik (get-ik :right r-new-pose)))
    (unless (and l-ik r-ik)
      (cpl-impl:fail 'manipulation-failed
                     :format-control
                     "Parking with 2 arms failed due to invalid IK solution(s)"))
    (cpl:par
      (execute-arm-trajectory :left (ik->trajectory (first l-ik)))
      (execute-arm-trajectory :right (ik->trajectory (first r-ik))))))

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
      (unless (tf:wait-for-transform
               *tf*
               :source-frame (tf:frame-id pose)
               :target-frame target-frame
               :time stamp
               :timeout 5.0)
        (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
      (cond ((string= (tf:frame-id pose) target-frame)
             (tf:transform-pose
              *tf*
              :pose pose-offsetted
              :target-frame target-frame))
            (t pose-offsetted)))))

(defun relative-linear-translation->ik (arm &key (x 0.0) (y 0.0) (z 0.0))
  (let* ((wrist-transform (tf:lookup-transform
                           *tf*
                           :time 0
                           :source-frame
                           (ecase arm
                             (:right "r_wrist_roll_link")
                             (:left "l_wrist_roll_link"))
                           :target-frame "/torso_lift_link"))
         (translated-pose (tf:make-pose-stamped
                           (tf:frame-id wrist-transform)
                           (tf:stamp wrist-transform)
                           (cl-transforms:v+
                            (cl-transforms:translation
                             wrist-transform)
                            (cl-transforms:make-3d-vector
                             x y z))
                           (cl-transforms:rotation
                            wrist-transform)))
         (translated-ik (first (pr2-manip-pm::get-ik arm translated-pose))))
    translated-ik))
