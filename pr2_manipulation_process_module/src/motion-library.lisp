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

(in-package :pr2-manip-pm)

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
  (execute-arm-trajectory side
                          ;; Computes the lifting trajectory for the `side' arm
                          (get-lifting-grasped-object-arm-trajectory side distance)) 
  (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed)))

(defun lift-grasped-object-with-both-arms (distance)
 "Executes a parallel lifting motion with both arms in order to lift
the object which is grasped with both arms at `distance' form the
supporting plane"
  (cpl-impl:par
    (execute-arm-trajectory :left
                            ;; Compute the lifting trajectory for the `left' arm.
                            (get-lifting-grasped-object-arm-trajectory :left distance))
    (execute-arm-trajectory :right
                            ;; Compute the lifting trajectory for the `right' arm.
                            (get-lifting-grasped-object-arm-trajectory :right distance))))

(defun put-down-grasped-object-with-single-arm (obj location side obstacles)
  (roslisp:ros-info (pr2-manip process-module) "Putting down object single-handedly.")
  (assert (and (rete-holds `(object-in-hand ,obj ,side))) ()
          "Object ~a needs to be in the gripper" obj)
  (clear-collision-objects)
  (sem-map-coll-env:publish-semantic-map-collision-objects)
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
                               (cl-transforms:orientation unhand-pose)))
         (put-down-solution (get-constraint-aware-ik side put-down-pose))
         (unhand-solution (get-constraint-aware-ik
                           side unhand-pose-stamped
                           :allowed-collision-objects (list "\"all\""))))
    (when (or (not put-down-solution) (not unhand-solution))
      (cpl:fail 'manipulation-pose-unreachable))
    (execute-move-arm-pose side pre-put-down-pose)
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
    (execute-arm-trajectory side (ik->trajectory (lazy-car put-down-solution)))
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
    (open-gripper side)
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
    (plan-knowledge:on-event (make-instance 'plan-knowledge:object-detached
                               :object obj
                               :link (ecase side
                                       (:right "r_gripper_r_finger_tip_link")
                                       (:left "l_gripper_r_finger_tip_link"))))
    (execute-arm-trajectory
     side (ik->trajectory (lazy-car unhand-solution)))
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))))

(defun put-down-grasped-object-with-both-arms (obj location)
  (roslisp:ros-info (pr2-manip process-module) "Putting down the grasped object with both arms.")
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
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
    ;; ... pose
    (cpl-impl:par
      (execute-arm-trajectory :left (ik->trajectory (lazy-car left-put-down-ik)))
      (execute-arm-trajectory :right (ik->trajectory (lazy-car right-put-down-ik))))
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
    ;; ... open
    (cpl-impl:par
      (open-gripper :left :position 0.04)
      (open-gripper :right :position 0.04))
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
    ;; ... go away.
    (cpl-impl:par
      (execute-arm-trajectory :left (ik->trajectory (lazy-car left-unhand-ik)))
      (execute-arm-trajectory :right (ik->trajectory (lazy-car right-unhand-ik))))
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
    ;; TODO(Georg): ask Lorenz about this, and get it running
    ;; (plan-knowledge:on-event (make-instance 'plan-knowledge:object-detached
    ;;                            :object obj
    ;;                            :link (ecase side
    ;;                                    (:right "r_gripper_r_finger_tip_link")
    ;;                                    (:left "l_gripper_r_finger_tip_link"))))
    ))

(defun standard-grasping (obj side obstacles)
  (roslisp:ros-info (pr2-manip process-module) "Opening gripper")
  (open-gripper side)
  (roslisp:ros-info (pr2-manip process-module) "Clearing collision map")
  (clear-collision-objects)
  (roslisp:ros-info (pr2-manip process-module) "Adding object as collision object")
  (register-collision-object obj)
  ;; Check if the object we want to grasp is not too big
  (let ((bb (desig-bounding-box obj)))
    (when bb
      (roslisp:ros-info
       (pr2-manip process-module) "Bounding box: ~a ~a ~a"
       (cl-transforms:x bb) (cl-transforms:y bb) (cl-transforms:z bb)))
    (when (and
           bb
           (or
            (> (cl-transforms:x bb)
               (cl-transforms:x *max-graspable-size*))
            (> (cl-transforms:y bb)
               (cl-transforms:y *max-graspable-size*))
            (> (cl-transforms:z bb)
               (cl-transforms:z *max-graspable-size*))))
      ;; TODO: Use a more descriptive condition here
      (error 'manipulation-pose-unreachable
             :format-control "No valid grasp pose found")))
  (roslisp:ros-info (pr2-manip process-module) "Registering semantic map objects")
  (sem-map-coll-env:publish-semantic-map-collision-objects)
  (dolist (obstacle (cut:force-ll obstacles))
    (register-collision-object obstacle))
  (let ((grasp-poses
          (lazy-take
           50
           (lazy-mapcan (lambda (grasp)
                          (let ((pre-grasp-pose
                                  (tool-goal-pose->wrist-goal-pose
                                   obj
                                   :tool (calculate-tool-pose
                                          grasp
                                          *grasp-approach-distance*)))
                                (grasp-pose
                                  (tool-goal-pose->wrist-goal-pose
                                   obj
                                   :tool grasp)))
                            ;; If we find IK solutions
                            ;; for both poses, yield them
                            ;; to the lazy list
                            (when (and (ecase side
                                         (:left
                                          (grasp-orientation-valid
                                           pre-grasp-pose
                                           (list *top-grasp* *left-grasp*)
                                           (list *right-grasp*)))
                                         (:right
                                          (grasp-orientation-valid
                                           pre-grasp-pose
                                           (list *top-grasp* *right-grasp*)
                                           (list *left-grasp*)))))
                              (let* ((pre-grasp-solution (lazy-car
                                                          (get-ik
                                                           side pre-grasp-pose)))
                                     (grasp-solution (lazy-car
                                                      (get-constraint-aware-ik
                                                       side grasp-pose
                                                       :allowed-collision-objects (list "\"all\"")))))
                                (when (and pre-grasp-solution grasp-solution)
                                  (roslisp:ros-info (pr2-manip process-module) "Found valid grasp ~a ~a"
                                                    pre-grasp-pose grasp-pose)
                                  `(((,pre-grasp-pose ,pre-grasp-solution)
                                     (,grasp-pose ,grasp-solution))))))))
                        (prog2
                            (roslisp:ros-info (pr2-manip process-module) "Planning grasp")
                            (get-point-cluster-grasps side obj)
                          (roslisp:ros-info (pr2-manip process-module) "Grasp planning finished"))))))
    (unless grasp-poses
      (cpl:fail 'manipulation-pose-unreachable
                :format-control "No valid grasp pose found"))
    (roslisp:ros-info (pr2-manip process-module) "Executing move-arm")
    (destructuring-bind ((pre-grasp pre-solution) (grasp grasp-solution)) (lazy-car grasp-poses)
      (declare (ignore grasp))
      (execute-move-arm-pose side pre-grasp)
      (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
      (execute-arm-trajectory side (ik->trajectory grasp-solution))
      (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
      (roslisp:ros-info (pr2-manip process-module) "Closing gripper")
      ;; (compliant-close-gripper side)
      (close-gripper side :max-effort 50.0)
      (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
      (check-valid-gripper-state side :safety-ik pre-solution))
    (roslisp:ros-info (pr2-manip process-module) "Attaching object to gripper")
    (plan-knowledge:on-event (make-instance 'plan-knowledge:object-attached
                               :object obj
                               :link (ecase side
                                       (:right "r_gripper_r_finger_tip_link")
                                       (:left "l_gripper_r_finger_tip_link"))
                               :side side))
    (assert-occasion `(object-in-hand ,obj ,side))))

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
    (format t "pre-grasp-pose ~a~%" pre-grasp-pose)
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

(defun grasp-object-with-both-arms (obj)
  "Grasp an object `obj' with both arms."
  ;; compute grasping poses and execute dual arm grasp
  (let ((obj (newest-effective-designator obj)))
    (multiple-value-bind (left-grasp-pose right-grasp-pose)
                         (compute-both-arm-grasping-poses obj)
                         (execute-both-arm-grasp left-grasp-pose right-grasp-pose)
                         ;; TODO(Georg): make this depend on the execution result
                         ;; update object designator, because it is now in the grippers
                         ;; TODO(Georg): this constant is pot-specific -> move it somewhere else
                         (update-picked-up-object-designator
                          obj 'desig-props:both-grippers :left 0.0))))

(defun execute-both-arm-grasp
  (grasping-pose-left grasping-pose-right)
  "Grasps an object with both grippers
simultaneously. `grasping-pose-left' and `grasping-pose-right' specify
the goal poses for the left and right gripper. Before closing the
grippers, both grippers will be opened, the pre-grasp positions (using
parameter *grasp-approach-distance*) will be commanded, and finally
the grasping positions (with offset of parameter *grasp-distance*)
will be commanded."
  ;; make sure that the desired tf transforms exist
  (cl-tf:wait-for-transform *tf*
                            :timeout 1.0
                            :time (tf:stamp grasping-pose-left)
                            :source-frame (tf:frame-id grasping-pose-left)
                            :target-frame "base_footprint")
  (cl-tf:wait-for-transform *tf*
                            :timeout 1.0
                            :time (tf:stamp grasping-pose-right)
                            :source-frame (tf:frame-id grasping-pose-right)
                            :target-frame "base_footprint")
  (let* ((grasping-pose-left-transform (cl-transforms:pose->transform grasping-pose-left))
         (grasping-pose-right-transform (cl-transforms:pose->transform grasping-pose-right))
         ;; get grasping poses and pre-poses for both sides
         (pre-grasp-pose-left
          (cl-tf:transform-pose *tf*
                                :pose (tf:pose->pose-stamped
                                       (tf:frame-id grasping-pose-left)
                                       (tf:stamp grasping-pose-left)
                                        ; this is adding a tool and an
                                        ; approach offset in
                                        ; tool-frame
                                       (cl-transforms:transform-pose
                                        grasping-pose-left-transform
                                        (cl-transforms:make-transform
                                         (cl-transforms:make-3d-vector
                                          (+ (- *grasp-distance*) (- *grasp-approach-distance*)) 0.0 0.0)
                                         (cl-transforms:make-identity-rotation))))
                                :target-frame "base_footprint"))
         (pre-grasp-pose-right
          (cl-tf:transform-pose *tf*
                                :pose (tf:pose->pose-stamped
                                       (tf:frame-id grasping-pose-right)
                                       (tf:stamp grasping-pose-right)
                                        ; this is adding a tool and an
                                        ; approach offset in
                                        ; tool-frame
                                       (cl-transforms:transform-pose
                                        grasping-pose-right-transform
                                        (cl-transforms:make-transform
                                         (cl-transforms:make-3d-vector
                                          (+ (- *grasp-distance*) (- *grasp-approach-distance*)) 0.0 0.0)
                                         (cl-transforms:make-identity-rotation))))
                                :target-frame "base_footprint"))
         (grasp-pose-left
          (cl-tf:transform-pose *tf*
                                :pose (tf:pose->pose-stamped
                                       (tf:frame-id grasping-pose-left)
                                       (tf:stamp grasping-pose-left)
                                        ; this is adding a tool and an
                                        ; approach offset in
                                        ; tool-frame
                                       (cl-transforms:transform-pose
                                        grasping-pose-left-transform
                                        (cl-transforms:make-transform
                                         (cl-transforms:make-3d-vector
                                          (- *grasp-distance*) 0.0 0.0)
                                         (cl-transforms:make-identity-rotation))))
                                :target-frame "base_footprint"))
         (grasp-pose-right
          (cl-tf:transform-pose *tf*
                                :pose (tf:pose->pose-stamped
                                       (tf:frame-id grasping-pose-right)
                                       (tf:stamp grasping-pose-right)
                                        ; this is adding a tool and an
                                        ; approach offset in
                                        ; tool-frame
                                       (cl-transforms:transform-pose
                                        grasping-pose-right-transform
                                        (cl-transforms:make-transform
                                         (cl-transforms:make-3d-vector
                                          (- *grasp-distance*) 0.0 0.0)
                                         (cl-transforms:make-identity-rotation))))
                                :target-frame "base_footprint"))
         ;; get seed-states for ik
         (left-seed-state (calc-seed-state-elbow-up :left))
         (right-seed-state (calc-seed-state-elbow-up :right))
         ;; get ik-solutions for all poses
         (pre-grasp-left-ik
          (lazy-car
           (get-ik :left pre-grasp-pose-left :seed-state left-seed-state)))
         (pre-grasp-right-ik
          (lazy-car
           (get-ik :right pre-grasp-pose-right :seed-state right-seed-state)))
         (grasp-left-ik
          (lazy-car
           (get-ik :left grasp-pose-left :seed-state left-seed-state)))
         (grasp-right-ik
          (lazy-car
           (get-ik :right grasp-pose-right :seed-state right-seed-state))))
    ;; check if left poses are left of right poses
    (when (< (cl-transforms:y (cl-transforms:origin pre-grasp-pose-left))
             (cl-transforms:y (cl-transforms:origin pre-grasp-pose-right)))
      (error 'manipulation-failed
             :format-control "Left pre-grasp pose right of right pre-grasp pose."))
    (when (< (cl-transforms:y (cl-transforms:origin grasp-pose-left))
             (cl-transforms:y (cl-transforms:origin grasp-pose-right)))
      (error 'manipulation-failed
             :format-control "Left grasp pose right of right grasp pose."))
    ;; check if all for all poses ik-solutions exist
    (unless pre-grasp-left-ik
      (error 'manipulation-pose-unreachable
             :format-control "Trying to grasp with both arms -> pre-grasp pose for the left arm is NOT reachable."))
    (unless pre-grasp-right-ik
      (error 'manipulation-pose-unreachable
             :format-control "Trying to grasp with both arms -> pre-grasp pose for the right arm is NOT reachable."))
    (unless grasp-left-ik
      (error 'manipulation-pose-unreachable
             :format-control "Trying to grasp with both arms -> grasp pose for the left arm is NOT reachable."))
    (unless grasp-right-ik
      (error 'manipulation-pose-unreachable
             :format-control "Trying to grasp with both arms -> grasp pose for the right arm is NOT reachable."))
    ;; simultaneously open both grippers
    (cpl-impl:par
     ;; only open the grippers to 50% to not hit the lid
     (open-gripper :left :position 0.04)
     (open-gripper :right :position 0.04))
    ;; simultaneously approach pre-grasp positions from both sides
    (let ((new-stamp (roslisp:ros-time)))
      (cpl-impl:par      
       (execute-arm-trajectory :left (ik->trajectory pre-grasp-left-ik :stamp new-stamp))
       (execute-arm-trajectory :right (ik->trajectory pre-grasp-right-ik :stamp new-stamp))))
    ;; simultaneously approach grasp positions from both sides
    (let ((new-stamp (roslisp:ros-time)))
      (cpl-impl:par
       (execute-arm-trajectory :left (ik->trajectory grasp-left-ik :stamp new-stamp))
       (execute-arm-trajectory :right (ik->trajectory grasp-right-ik :stamp new-stamp))))
    ;; simultaneously close both grippers
    (cpl-impl:par
     (close-gripper :left :max-effort 50)
     (close-gripper :right :max-effort 50))))

(defun check-valid-gripper-state (side &key (min-position 0.01) safety-ik)
  (when (< (get-gripper-state side) min-position)
    (clear-collision-objects)
    (open-gripper side)
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
    (when safety-ik
      (execute-arm-trajectory side (ik->trajectory safety-ik))
      (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed)))
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
                               :allowed-collision-objects (list "\"all\"")))
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))))