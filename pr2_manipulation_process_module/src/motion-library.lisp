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
;;;       Technische Universitaet Bremen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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
                                   0.2 -0.55 1.00)
                                  (cl-transforms:euler->quaternion :ay (/ pi 2))))
(defparameter *carry-pose-left* (tf:make-pose-stamped
                                 "/base_footprint" 0.0
                                 (cl-transforms:make-3d-vector
                                  0.2 0.55 1.00)
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

(defparameter *max-graspable-size* (cl-transforms:make-3d-vector 0.15 0.15 0.30))

(defun execute-grasp (&key pregrasp-pose grasp-solution
                        side (gripper-open-pos 0.08) (gripper-close-pos 0.0))
  (assert (and pregrasp-pose grasp-solution) ()
          "Unspecified parameter in execute-grasp.")
  (roslisp:ros-info (pr2-manipulation-process-module)
                    "Executing pregrasp for side ~a~%" side)
  (execute-move-arm-pose side pregrasp-pose)
  (roslisp:ros-info (pr2-manipulation-process-module) "Opening gripper")
  (open-gripper side :max-effort 50.0 :position gripper-open-pos)
  (roslisp:ros-info (pr2-manipulation-process-module)
                    "Executing grasp for side ~a~%" side)
  (execute-arm-trajectory side (ik->trajectory grasp-solution))
  (roslisp:ros-info (pr2-manipulation-process-module) "Closing gripper")
  (close-gripper side :max-effort 50.0 :position gripper-close-pos))

(defun execute-putdown (&key pre-putdown-pose putdown-solution unhand-solution
                          side (gripper-open-pos 0.08) allowed-collision-objects)
  (assert (and pre-putdown-pose putdown-solution unhand-solution) ()
          "Unspecified parameter in execute-putdown.")
  (roslisp:ros-info (pr2-manipulation-process-module)
                    "Executing pre-putdown for side ~a~%" side)
  (roslisp:publish (roslisp:advertise "/preputdownpose"
                                      "geometry_msgs/PoseStamped")
                   (tf:pose-stamped->msg pre-putdown-pose))
  (execute-move-arm-pose side pre-putdown-pose
                         :allowed-collision-objects allowed-collision-objects)
  (roslisp:ros-info (pr2-manipulation-process-module)
                    "Executing putdown for side ~a~%" side)
  (execute-arm-trajectory side (ik->trajectory putdown-solution))
  (roslisp:ros-info (pr2-manipulation-process-module) "Opening gripper")
  (open-gripper side :max-effort 50.0 :position gripper-open-pos)
  (execute-arm-trajectory side (ik->trajectory unhand-solution)))

(defun get-lifting-grasped-object-arm-trajectory (side distance)
  "Returns the lifting trajectory for the `side' robot arm in order to
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
                       (cl-transforms:v+ (cl-transforms:translation wrist-transform)
                                         (cl-transforms:make-3d-vector 0 0 distance))
                       (cl-transforms:rotation wrist-transform)))
         (lifted-pose-ik (get-ik side lifted-pose)))
    (unless lifted-pose-ik
      (error 'manipulation-pose-unreachable
             :format-control "Lifted pose for " side " arm unreachable !"))
    (ik->trajectory (lazy-car lifted-pose-ik))))

(defun lift-grasped-object-with-one-arm (side distance)
  "Executes a lifting motion with the `side' arm which grasped the
object in order to lift it at `distance' form the supporting plane"
  (execute-arm-trajectory
   side
   ;; Computes the lifting trajectory for the `side' arm
   (get-lifting-grasped-object-arm-trajectory side distance)))

(defun lift-grasped-object-with-both-arms (distance)
  "Executes a parallel lifting motion with both arms in order to lift
the object which is grasped with both arms at `distance' form the
supporting plane"
  (cpl-impl:par
    (execute-arm-trajectory :left
                            ;; Compute the lifting trajectory for the
                            ;; `left' arm.
                            (get-lifting-grasped-object-arm-trajectory
                             :left distance))
    (execute-arm-trajectory :right
                            ;; Compute the lifting trajectory for the
                            ;; `right' arm.
                            (get-lifting-grasped-object-arm-trajectory
                             :right distance))))

(defun put-down-grasped-object-with-single-arm (obj location side obstacles)
  (roslisp:ros-info (pr2-manipulation-process-module) "Putting down object single-handedly.")
  (assert (holds `(object-in-hand ,obj ,side))() 
          "Object ~a needs to be in the gripper" obj)
  (clear-collision-objects)
  (dolist (obstacle (cut:force-ll obstacles))
    (register-collision-object obstacle))
  (let* ((put-down-pose (calculate-put-down-pose obj location))
         (pre-put-down-pose (tf:copy-pose-stamped
                             put-down-pose
                             :origin (cl-transforms:v+
                                      (cl-transforms:origin put-down-pose)
                                      (cl-transforms:make-3d-vector 0 0 *pre-put-down-distance*))))
         (unhand-pose (cl-transforms:transform-pose
                       (cl-transforms:reference-transform put-down-pose)
                       (cl-transforms:make-pose
                        (cl-transforms:make-3d-vector (- *pre-put-down-distance*) 0 0)
                        (cl-transforms:make-identity-rotation))))
         (unhand-pose-stamped (tf:make-pose-stamped
                               (tf:frame-id put-down-pose) (tf:stamp put-down-pose)
                               (cl-transforms:origin unhand-pose)
                               (cl-transforms:orientation unhand-pose))))
    (let ((current-time (roslisp:ros-time)))
      (tf:wait-for-transform *tf* :time current-time
                             :source-frame (tf:frame-id put-down-pose)
                             :target-frame "/torso_lift_link")
      (let* ((put-down-pose-tll (tf:transform-pose
                                 *tf*
                                 :pose (tf:copy-pose-stamped
                                        put-down-pose
                                        :stamp current-time)
                                 :target-frame "/torso_lift_link"))
             (put-down-solution (get-ik side put-down-pose-tll
                                        :seed-state (calc-seed-state-elbow-up
                                                     side)))
             (unhand-solution (get-ik;constraint-aware-ik
                               side unhand-pose-stamped)))
        ;;:allowed-collision-objects (list "\"all\""))))
        ;(execute-move-arm-pose side pre-put-down-pose)
        ;; (format t "::: ~a~%" put-down-solution)
        ;; (roslisp:publish
        ;;  (roslisp:advertise "/putdownpose" "geometry_msgs/PoseStamped")
        ;;  (tf:pose-stamped->msg put-down-pose))
        ;; (when (or (not put-down-solution) (not unhand-solution))
        ;;   (cpl:fail 'manipulation-pose-unreachable))
        ;; (execute-move-arm-pose side pre-put-down-pose)
        ;; (execute-move-arm-pose side put-down-pose-tll)
        ;(let ((ik (first (get-ik side pre-put-down-pose))))
        ;  (cond (ik (execute-arm-trajectory side (ik->trajectory ik)))
        ;        (t (cpl:fail 'cram-plan-failures:manipulation-failed))))
        ;(execute-arm-trajectory side (ik->trajectory (lazy-car put-down-solution)))
        (open-gripper side)
        (plan-knowledge:on-event
         (make-instance
          'plan-knowledge:object-detached
          :side side
          :object obj
          :link (ecase side
                  (:right "r_gripper_r_finger_tip_link")
                  (:left "l_gripper_r_finger_tip_link"))))
;        (execute-arm-trajectory
;         side (ik->trajectory (lazy-car unhand-solution)))
        ))))

(defun put-down-grasped-object-with-both-arms (obj location)
  (roslisp:ros-info (pr2-manipulation-process-module) "Putting down the grasped object with both arms.")
  ;; TODO(Georg): Talk to Lorenz about this, and how to get it running
  ;; (assert (and (rete-holds `(object-in-hand ,obj ,side))) ()
  ;;         "Object ~a needs to be in the gripper" obj)
  (let* ((goal-transform-obj (calc-goal-transform-for-picked-object obj location))
         ;; get put down poses for the handles
         ;; T_centerPot_in_base * T_leftHandle_in_centerPot * T_tool_in_wrist_INVERSE
         ;; result is a pose and still needs to get poseStamped
         ;; NOTE: intended frame_id: /base_footprint
         (left-put-down-pose (cl-transforms:transform-pose
                              (cl-transforms:pose->transform
                               (cl-transforms:transform-pose
                                goal-transform-obj
                                *pot-relative-left-handle-transform*))
                              (cl-transforms:make-pose
                               (cl-transforms:make-3d-vector
                                (+ (- *grasp-distance*)) 0.0 0.0)
                               (cl-transforms:make-identity-rotation))))
         ;; T_centerPot_in_base * T_rightHandle_in_centerPot * T_tool_in_wrist_INVERSE
         ;; result is a pose and still needs to get poseStamped
         ;; NOTE: intended frame_id: /base_footprint
         (right-put-down-pose (cl-transforms:transform-pose
                               (cl-transforms:pose->transform
                                (cl-transforms:transform-pose
                                 goal-transform-obj
                                 *pot-relative-right-handle-transform*))
                               (cl-transforms:make-pose
                                (cl-transforms:make-3d-vector
                                 (+ (- *grasp-distance*)) 0.0 0.0)
                                (cl-transforms:make-identity-rotation))))
         ;; make 'em stamped poses
         (left-put-down-stamped-pose (cl-tf:make-pose-stamped
                                      "/base_footprint"
                                      (cl-tf:stamp
                                       (reference (current-desig location)))
                                      (cl-transforms:origin left-put-down-pose)
                                      (cl-transforms:orientation left-put-down-pose)))
         (right-put-down-stamped-pose (cl-tf:make-pose-stamped
                                       "/base_footprint"
                                       (cl-tf:stamp
                                        (reference (current-desig location)))
                                       (cl-transforms:origin right-put-down-pose)
                                       (cl-transforms:orientation right-put-down-pose)))
         ;; calc pre-put-down poses
         (left-pre-put-down-pose (tf:copy-pose-stamped
                                  left-put-down-stamped-pose
                                  :origin (cl-transforms:v+
                                           (cl-transforms:origin left-put-down-stamped-pose)
                                           (cl-transforms:make-3d-vector 0 0 *pre-put-down-distance*))))
         (right-pre-put-down-pose (tf:copy-pose-stamped
                                   right-put-down-stamped-pose
                                   :origin (cl-transforms:v+
                                            (cl-transforms:origin right-put-down-stamped-pose)
                                            (cl-transforms:make-3d-vector 0 0 *pre-put-down-distance*))))
         ;; calc unhand poses
         (left-unhand-pose (cl-transforms:transform-pose
                            (cl-transforms:reference-transform left-put-down-stamped-pose)
                            (cl-transforms:make-pose
                             (cl-transforms:make-3d-vector (- *pre-put-down-distance*) 0 0)
                             (cl-transforms:make-identity-rotation))))
         (right-unhand-pose (cl-transforms:transform-pose
                             (cl-transforms:reference-transform right-put-down-stamped-pose)
                             (cl-transforms:make-pose
                              (cl-transforms:make-3d-vector (- *pre-put-down-distance*) 0 0)
                              (cl-transforms:make-identity-rotation))))
         (left-unhand-pose-stamped (tf:make-pose-stamped
                                    (tf:frame-id left-put-down-stamped-pose)
                                    (tf:stamp left-put-down-stamped-pose)
                                    (cl-transforms:origin left-unhand-pose)
                                    (cl-transforms:orientation left-unhand-pose)))
         (right-unhand-pose-stamped (tf:make-pose-stamped 
                                     (tf:frame-id right-put-down-stamped-pose)
                                     (tf:stamp right-put-down-stamped-pose)
                                     (cl-transforms:origin right-unhand-pose)
                                     (cl-transforms:orientation right-unhand-pose)))
         ;; ask for ik solutions for all 6 poses (3 for each arm):
         ;; pre-put-down, put-down, and unhand pose
         (left-put-down-ik (get-ik :left left-put-down-stamped-pose))
         (right-put-down-ik (get-ik :right right-put-down-stamped-pose))
         (left-pre-put-down-ik (get-ik :left left-pre-put-down-pose))
         (right-pre-put-down-ik (get-ik :right right-pre-put-down-pose))
         (left-unhand-ik (get-ik :left left-unhand-pose-stamped))
         (right-unhand-ik (get-ik :right right-unhand-pose-stamped)))
    ;; check if left and right are not switched
    (when (< (cl-transforms:y (cl-transforms:origin left-put-down-pose))
             (cl-transforms:y (cl-transforms:origin right-put-down-pose)))
      (error 'manipulation-failed
             :format-control "Left put-down pose right of right put-down pose."))
    ;; check if IKs exist
    (unless left-put-down-ik
      (error 'manipulation-pose-unreachable
             :format-control "No ik solution for left-put-down-pose"))
    (unless right-put-down-ik
      (error 'manipulation-pose-unreachable
             :format-control "No ik solution for right-put-down-pose"))
    (unless left-pre-put-down-ik
      (error 'manipulation-pose-unreachable
             :format-control "No ik solution for left-pre-put-down-pose"))
    (unless right-pre-put-down-ik
      (error 'manipulation-pose-unreachable
             :format-control "No ik solution for right-pre-put-down-pose"))
    (unless left-unhand-ik
      (error 'manipulation-pose-unreachable
             :format-control "No ik solution for left-unhand-pose"))
    (unless right-unhand-ik
      (error 'manipulation-pose-unreachable
             :format-control "No ik solution for right-unhand-pose"))
    ;; Execute the put down motion...
    ;; ... pre-pose
    (cpl-impl:par
      (execute-arm-trajectory :left (ik->trajectory (lazy-car left-pre-put-down-ik)))
      (execute-arm-trajectory :right (ik->trajectory (lazy-car right-pre-put-down-ik))))
    ;; ... pose
    (cpl-impl:par
      (execute-arm-trajectory :left (ik->trajectory (lazy-car left-put-down-ik)))
      (execute-arm-trajectory :right (ik->trajectory (lazy-car right-put-down-ik))))
    ;; ... open
    (cpl-impl:par
      (open-gripper :left :position 0.04)
      (open-gripper :right :position 0.04))
    ;; ... go away.
    (cpl-impl:par
      (execute-arm-trajectory :left (ik->trajectory (lazy-car left-unhand-ik)))
      (execute-arm-trajectory :right (ik->trajectory (lazy-car right-unhand-ik))))
    ;; TODO(Georg): ask Lorenz about this, and get it running
    ;; (plan-knowledge:on-event (make-instance 'plan-knowledge:object-detached
    ;;                            :object obj
    ;;                            :link (ecase side
    ;;                                    (:right "r_gripper_r_finger_tip_link")
    ;;                                    (:left "l_gripper_r_finger_tip_link"))))
    ))

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
           (roslisp:ros-error (pr2-manipulation-process-module check-valid-gripper-state)
                              "Only one of `safety-ik' and `safety-pose' can be specified."))
          (safety-ik
           (execute-arm-trajectory side (ik->trajectory safety-ik)))
          (safety-pose
           (execute-move-arm-pose side safety-pose))
          (t
           (roslisp:ros-error
            (pr2-manipulation-process-module check-valid-gripper-state)
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
                               :allowed-collision-objects (list "\"all\""))
        (execute-move-arm-pose side carry-pose
                               :allowed-collision-objects (list "\"all\"")))))

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

(defun shrug-arms ()
  "Moves the arms in an upper side-ways position so that they don't
interfere with one another when manipulation is one."
  (let* ((shrug-pose-right (tf:make-pose-stamped
                            "torso_lift_link"
                            0.0
                            (tf:make-3d-vector 0.3 -0.8 0.0)
                            (tf:euler->quaternion :az (/ pi -8))))
         (shrug-pose-left (tf:make-pose-stamped
                           "torso_lift_link"
                           0.0
                           (tf:make-3d-vector 0.3 0.8 0.0)
                           (tf:euler->quaternion :az (/ pi 8))))
         (ik-right (first (get-ik :right shrug-pose-right
                                  :seed-state (calc-seed-state-elbow-up :right))))
         (ik-left (first (get-ik :left shrug-pose-left
                                 :seed-state (calc-seed-state-elbow-up :left)))))
    (when (and ik-right ik-left)
      (cram-language::par
        (execute-arm-trajectory :right (ik->trajectory ik-right))
        (execute-arm-trajectory :left (ik->trajectory ik-left))))))

(defun relative-grasp-pose (pose pose-offset)
  (let ((stamp (roslisp:ros-time)))
    (tf:wait-for-transform *tf*
                           :time stamp
                           :source-frame (tf:frame-id pose)
                           :target-frame "/torso_lift_link")
  (let* ((grasp-pose (tf:pose->pose-stamped
                      (tf:frame-id pose)
                      stamp
                      (cl-transforms:transform-pose
                       (tf:pose->transform pose)
                       pose-offset)))
         (grasp-pose-tll (tf:transform-pose
                          *tf*
                          :pose grasp-pose
                          :target-frame "/torso_lift_link")))
    grasp-pose-tll)))

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
