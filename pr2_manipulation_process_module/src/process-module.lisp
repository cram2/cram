;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :pr2-manip-pm)

(define-condition move-arm-no-ik-solution (manipulation-failure) ())
(define-condition move-arm-ik-link-in-collision (manipulation-failure) ())

(defparameter *grasp-approach-distance* 0.10
  "Distance to approach the object. This parameter is used to
  calculate the pose to approach with move_arm.")
(defparameter *grasp-distance* 0.18
  "Tool length to calculate the pre-grasp pose, i.e. the pose at which
  the gripper is closed.")
(defparameter *pre-put-down-distance* 0.07
  "Distance above the goal before putting down the object")

(defparameter *max-graspable-size* (cl-transforms:make-3d-vector 0.15 0.15 0.30))

(defvar *gripper-action-left* nil)
(defvar *gripper-action-right* nil)
(defvar *gripper-grab-action-left* nil)
(defvar *gripper-grab-action-right* nil)

(defvar *trajectory-action-left* nil)
(defvar *trajectory-action-right* nil)
(defvar *trajectory-action-both* nil)
(defvar *trajectory-action-torso* nil)
(defvar *move-arm-right* nil)
(defvar *move-arm-left* nil)

(defvar *joint-state-sub* nil)

(defun init-pr2-manipulation-process-module ()
  (setf *gripper-action-left* (actionlib:make-action-client
                               "/l_gripper_controller/gripper_action"
                               "pr2_controllers_msgs/Pr2GripperCommandAction"))
  (setf *gripper-action-right* (actionlib:make-action-client
                                "/r_gripper_controller/gripper_action"
                                "pr2_controllers_msgs/Pr2GripperCommandAction"))
  (setf *gripper-grab-action-left* (actionlib:make-action-client
                                    "/l_gripper_sensor_controller/grab"
                                    "pr2_gripper_sensor_msgs/PR2GripperGrabAction"))
  (setf *gripper-grab-action-right* (actionlib:make-action-client
                                     "/r_gripper_sensor_controller/grab"
                                     "pr2_gripper_sensor_msgs/PR2GripperGrabAction"))
  (setf *trajectory-action-left* (actionlib:make-action-client
                                  "/l_arm_controller/joint_trajectory_generator"
                                  "pr2_controllers_msgs/JointTrajectoryAction"))
  (setf *trajectory-action-right* (actionlib:make-action-client
                                   "/r_arm_controller/joint_trajectory_generator"
                                   "pr2_controllers_msgs/JointTrajectoryAction"))
  (setf *trajectory-action-both* (actionlib:make-action-client
                                   "/both_arms_controller/joint_trajectory_action"
                                   "pr2_controllers_msgs/JointTrajectoryAction"))
  (setf *trajectory-action-torso* (actionlib:make-action-client
                                   "/torso_controller/joint_trajectory_action"
                                   "pr2_controllers_msgs/JointTrajectoryAction"))
  (setf *move-arm-right* (actionlib:make-action-client
                          "/move_right_arm"
                          "arm_navigation_msgs/MoveArmAction"))
  (setf *move-arm-left* (actionlib:make-action-client
                         "/move_left_arm"
                         "arm_navigation_msgs/MoveArmAction"))
  (setf *joint-state-sub* (roslisp:subscribe
                           "/joint_states" "sensor_msgs/JointState"
                           (lambda (msg)
                             (setf *joint-state* msg))))
  ;; Initialize the planning scene to make get_ik and friends work.
  (when (roslisp:wait-for-service "/environment_server/set_planning_scene_diff" 0.2)
    (roslisp:call-service
     "/environment_server/set_planning_scene_diff"
     "arm_navigation_msgs/SetPlanningSceneDiff")))

(register-ros-init-function init-pr2-manipulation-process-module)

(defgeneric call-action (action &rest params))

(defmethod call-action ((action-sym t) &rest params)
  (roslisp:ros-info (pr2-manip process-module)
                    "Unimplemented operation `~a' with parameters ~a. Doing nothing."
                    action-sym params)
  (sleep 0.5))

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(defmethod call-action :around (action-sym &rest params)
  (roslisp:ros-info (pr2-manip process-module) "Executing manipulation action ~a ~a."
                    action-sym params)
  (call-next-method)
  (roslisp:ros-info (pr2-manip process-module) "Manipulation action done."))

(def-action-handler container-opened (handle side)
  (let* ((handle-pose (designator-pose (newest-valid-designator handle)))
         (new-object-pose (open-drawer handle-pose side)))
    (update-object-designator-pose handle new-object-pose))
  (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed)))

(def-action-handler container-closed (handle side)
  (let* ((handle-pose (designator-pose (newest-valid-designator handle)))
         (new-object-pose (close-drawer handle-pose side)))
    (update-object-designator-pose handle new-object-pose))
  (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed)))

(def-action-handler park (obj side &optional obstacles)
  (roslisp:ros-info (pr2-manip process-module) "Park arms ~a ~a"
                    obj side)
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

(def-action-handler lift (side distance)
  (if (eq side :both)
      (lift-grasped-object-with-both-arms distance)
      (lift-grasped-object-with-one-arm side distance)))

(defun get-lifting-grasped-object-arm-trajectory (side distance)
  "Returns the lifting trajectory for the `side' robot arm in order to
lift the grasped object at the `distance' from the supporting plane."
  (let* ((wrist-transform (tf:lookup-transform
                           *tf*
                           :time 0
                           :source-frame (ecase side
                                           (:right "r_wrist_roll_link")
                                           (:left "l_wrist_roll_link"))
                           :target-frame "/base_footprint"))
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

(def-action-handler grasp (object-type obj side obstacles)
  "Selects and calls the appropriate grasping functionality based on
the given object type."
  (cond ((eq object-type 'desig-props:pot)
         (grasp-object-with-both-arms obj))
        ((eq object-type 'desig-props:handled)
         (grab-object-with-handles obj side))
        (t (standard-grasping obj side obstacles))))

(def-action-handler put-down (object-designator location side obstacles)
  "Delegates the type of the put down action which suppose to be executed
for the currently type of grasped object."
  (cond ((eq (desig-prop-value object-designator 'desig-props:type) 'desig-props:pot)
         (put-down-grasped-object-with-both-arms object-designator location))
        (t (put-down-grasped-object-with-single-arm object-designator location side obstacles))))

(defun grab-object-with-handles (obj side)
  "Grasp an object `obj' on one of its handles with the specified gripper side `side'. This includes going into pregrasp for the nearest handle, opening the gripper, going into the grasp position, closing the gripper and lifting the object by 0.2m by default."
  (with-desig-props (handles) obj
		    ;; Check if there are handles
		    (assert (> (length handles) 0) () "Object ~a needs at least one handle." obj)
		    ;, Get the nearest (atm the first) handle
		    (let ((nearest-handle (nearest-handle-for-side obj side)))
		      ;; Go into pregrasp for that handle on side `side'
		      (pregrasp-handled-object-with-relative-location obj side nearest-handle)
          ;; Open gripper
          (open-gripper side :position 0.04)
          ;; Go into grasp pose
          (grasp-handled-object-with-relative-location obj side nearest-handle)
          ;; Close gripper
          (close-gripper side :position 0.01)
          ;; Lift object
          (lift-handled-object-with-relative-location obj side nearest-handle))))

(defun taxi-handled-object (obj side relative-handle-loc &key (relative-gripper-pose (tf:make-identity-pose)))
  "Commutes the arm to a certain absolute pose. The target pose is determined through the absolute object pose of object `obj', the relative object handle location `relative-handle-loc' and the relative gripper pose `relative-gripper-pose' w.r.t. the handle. The relative gripper pose defaults to an identity pose."
  (let ((absolute-loc (object-handle-absolute obj relative-handle-loc :handle-offset-pose relative-gripper-pose)))
    (let ((move-ik (get-ik side (desig-prop-value absolute-loc 'desig-props:pose))))
      (assert (not (eq move-ik nil)) () "IK solution generation for side ~a failed during taxi for location designator ~a.~%" side absolute-loc)
      (let ((move-trajectory (ik->trajectory (first move-ik) :duration 5.0)))
        (assert (not (eq move-trajectory nil)) () "Trajectory generation for side ~a failed during taxi for location designator ~a.~%" side move-trajectory)
        (multiple-value-bind (result resultflag) (execute-arm-trajectory side move-trajectory)
          resultflag)))))

(defun lift-handled-object-with-relative-location (obj side relative-handle-loc &key (lift-height 0.2))
  "Moves the gripper side `side' into the lift position with respect to the object's `obj' handle `handle'. The lift height is specified through the key parameter `lift-height'."
  (taxi-handled-object obj side relative-handle-loc :relative-gripper-pose 
                       (cl-transforms:transform-pose
                        (tf:make-transform
                         (tf:make-3d-vector 0.12 0.0 0.0)
                         (tf:euler->quaternion :az pi :ax (/ pi 2)))
                        (tf:make-pose
                         (tf:make-3d-vector 0.0 0.0 lift-height)
                         (tf:make-identity-rotation)))))


(defun grasp-handled-object-with-relative-location (obj side relative-handle-loc)
  "Moves the gripper side `side' into the grasp position with respect to the object's `obj' handle `handle'."
  (taxi-handled-object obj side relative-handle-loc :relative-gripper-pose
                       (tf:make-pose (tf:make-3d-vector 0.12 0.0 0.0) (tf:euler->quaternion :az pi :ax (/ pi 2)))))

(defun pregrasp-handled-object-with-relative-location (obj side relative-handle-loc)
  "Moves the gripper side `side' into the pregrasp position with respect to the object's `obj' handle `handle'."
  (taxi-handled-object obj side relative-handle-loc :relative-gripper-pose
                       (tf:make-pose
                        (tf:make-3d-vector 0.2 0.0 0.0)
                        (tf:euler->quaternion :az pi :ax (/ pi 2)))))


(defun object-handle-absolute (obj handle &key (handle-offset-pose (tf:make-identity-pose)))
  "Transforms the relative handle location `handle' of object `obj' into the object's coordinate system and returns the appropriate location designator. The optional parameter `handle-offset-pose' is applied to the handle pose before the absolute object pose is applied."
  (let* ((absolute-object-loc (desig-prop-value obj 'desig-props:location))
	 (absolute-object-pose-stamped (desig-prop-value absolute-object-loc 'desig-props:pose)))
    (let ((relative-handle-pose
	   (cl-transforms:transform-pose
	    (tf:pose->transform
	     (desig-prop-value handle 'desig-props:pose))
	     handle-offset-pose)))
      (make-designator 'location `((desig-props:pose
				    ,(tf:pose->pose-stamped
				      (tf:frame-id absolute-object-pose-stamped)
				      (tf:stamp absolute-object-pose-stamped)
				      (cl-transforms:transform-pose
				       (tf:pose->transform absolute-object-pose-stamped)
				       relative-handle-pose))))))))

(defun nearest-handle-for-side (obj side)
  "Get the nearest handle location designator on object `obj' in respect to the chosen gripper side `side'."
  ;; TODO(winkler): Implement *actual* calculations concerning distance here. Atm, this always returns the first handle on the object. This is no problem as long as we only have one handle.
  (with-desig-props (handles) obj
		    (first handles)))

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
      (when (< (get-gripper-state side) 0.01)
        (clear-collision-objects)
        (open-gripper side)
        (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
        (execute-arm-trajectory side (ik->trajectory pre-solution))
        (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
        (cpl:fail 'object-lost)))
    ;; TODO: Check if gripper is not completely closed to make sure that we are holding the object
    (roslisp:ros-info (pr2-manip process-module) "Attaching object to gripper")
    (plan-knowledge:on-event (make-instance 'plan-knowledge:object-attached
                               :object obj
                               :link (ecase side
                                       (:right "r_gripper_r_finger_tip_link")
                                       (:left "l_gripper_r_finger_tip_link"))
                               :side side))
    (assert-occasion `(object-in-hand ,obj ,side))))

(defun execute-goal (server goal)
  (multiple-value-bind (result status)
      (actionlib:call-goal server goal)
    (unless (eq status :succeeded)
      (cpl-impl:fail 'manipulation-failed :format-control "Manipulation failed"))
    result))

(defun execute-torso-command (trajectory)
  (actionlib:call-goal
   *trajectory-action-torso*
   (actionlib:make-action-goal
       *trajectory-action-torso*
     :trajectory (remove-trajectory-joints #("torso_lift_joint") trajectory :invert t))))

(defun execute-arm-trajectory (side trajectory)
  (let ((action (ecase side
                  (:left
                   (switch-controller #("l_arm_controller") #("both_arms_controller"))
                   *trajectory-action-left*)
                  (:right
                   (switch-controller #("r_arm_controller") #("both_arms_controller"))
                   *trajectory-action-right*)
                  (:both
                   (switch-controller
                    #("both_arms_controller") #("l_arm_controller" "r_arm_controller"))
                   *trajectory-action-both*))))
    (actionlib:call-goal
     action
     (actionlib:make-action-goal
         action
       :trajectory (remove-trajectory-joints #("torso_lift_joint") trajectory)))))

(defun execute-move-arm-pose (side pose &key
                                          (planners '(:chomp :ompl))
                                          allowed-collision-objects)
  "Executes move arm. It goes through the list of planners specified
by `planners' until one succeeds."
  (flet ((execute-action (planner)
           (let ((action (ecase side
                           (:left *move-arm-left*)
                           (:right *move-arm-right*)))
                 (pose-msg (tf:pose-stamped->msg pose)))
             (roslisp:with-fields (header
                                   (position (position pose))
                                   (orientation (orientation pose)))
                 pose-msg
               (roslisp:with-fields ((val (val error_code)))
                   (actionlib:call-goal
                    action
                    (actionlib:make-action-goal
                        action
                      planner_service_name (ecase planner
                                             (:chomp "/chomp_planner_longrange/plan_path")
                                             (:ompl "/ompl_planning/plan_kinematic_path")
                                             (:stomp "/stomp_motion_planner/plan_path"))
                      (group_name motion_plan_request) (ecase side
                                                         (:right "right_arm")
                                                         (:left "left_arm"))
                      (num_planning_attempts motion_plan_request) 1
                      (planner_id motion_plan_request) ""
                      (allowed_planning_time motion_plan_request) 5.0
                      (expected_path_duration motion_plan_request) 5.0

                      (position_constraints goal_constraints motion_plan_request)
                      (vector
                       (roslisp:make-msg
                        "arm_navigation_msgs/PositionConstraint"
                        header header
                        link_name (ecase side
                                    (:left "l_wrist_roll_link")
                                    (:right "r_wrist_roll_link"))
                        position position
                        (type constraint_region_shape) (roslisp-msg-protocol:symbol-code
                                                        'arm_navigation_msgs-msg:shape
                                                        :box)
                        (dimensions constraint_region_shape) #(0.01 0.01 0.01)
                        (w constraint_region_orientation) 1.0
                        weight 1.0))

                      (orientation_constraints goal_constraints motion_plan_request)
                      (vector
                       (roslisp:make-msg
                        "arm_navigation_msgs/OrientationConstraint"
                        header header
                        link_name (ecase side
                                    (:left "l_wrist_roll_link")
                                    (:right "r_wrist_roll_link"))
                        orientation orientation
                        absolute_roll_tolerance 0.01
                        absolute_pitch_tolerance 0.01
                        absolute_yaw_tolerance 0.01
                        weight 1.0))

                      operations (make-collision-operations side (cons "\"attached\"" allowed-collision-objects))
                      disable_collision_monitoring t)
                    :result-timeout 4.0)
                 val)))))

    (case (reduce (lambda (result planner)
                    (declare (ignore result))
                    (let ((val (execute-action planner)))
                      (roslisp:ros-info (pr2-manip process-module)
                                        "Move arm returned with ~a"
                                        val)
                      (when (eql val 1)
                        (let ((goal-in-arm (tf:transform-pose
                                            *tf* :pose pose
                                                 :target-frame (ecase side
                                                                 (:left "l_wrist_roll_link")
                                                                 (:right "r_wrist_roll_link")))))
                          (when (or (> (cl-transforms:v-norm (cl-transforms:origin goal-in-arm))
                                       0.015)
                                    (> (cl-transforms:normalize-angle
                                        (nth-value 1 (cl-transforms:quaternion->axis-angle
                                                      (cl-transforms:orientation goal-in-arm))))
                                       0.03))
                            (error 'manipulation-failed)))
                        (return-from execute-move-arm-pose t))))
                  planners :initial-value nil)
      (-31 (error 'manipulation-pose-unreachable :result (list side pose)))
      (-33 (error 'manipulation-pose-occupied :result (list side pose)))
      (t (error 'manipulation-failed :result (list side pose))))))

(defun compliant-close-gripper (side)
  ;; (roslisp:call-service
  ;;  (ecase side
  ;;   (:right "/r_reactive_grasp/compliant_close")
  ;;   (:left "/l_reactive_grasp/compliant_close"))
  ;;  'std_srvs-srv:Empty)
  (let ((action (ecase side
                  (:right *gripper-grab-action-right*)
                  (:left *gripper-grab-action-left*))))
    (execute-goal
     action
     (actionlib:make-action-goal action
       (hardness_gain command) 0.03))))

(defun close-gripper (side &key (max-effort 100.0) (position 0.0))
  (let ((client (ecase side
                  (:right *gripper-action-right*)
                  (:left *gripper-action-left*))))
    (actionlib:send-goal-and-wait
     client (actionlib:make-action-goal client
              (position command) position
              (max_effort command) max-effort)
     :result-timeout 1.0)))

(defun open-gripper (side &key (max-effort 100.0) (position 0.085))
  (let ((client (ecase side
                  (:right *gripper-action-right*)
                  (:left *gripper-action-left*))))
    (prog1
        (actionlib:send-goal-and-wait
         client (actionlib:make-action-goal client
                  (position command) position
                  (max_effort command) max-effort)
         :result-timeout 1.0)
      (let ((obj (var-value
                  '?obj
                  (lazy-car
                   (rete-holds `(object-in-hand ?obj ,side))))))
        (unless (is-var obj)
          (retract-occasion `(object-in-hand ,obj ,side)))))))

(defclass manipulated-perceived-object (perceived-object) ())

(defun update-object-designator-pose (object-designator new-object-pose)
  (equate object-designator
          (perceived-object->designator
           object-designator
           (make-instance 'manipulated-perceived-object
             :pose new-object-pose
             :probability 1.0))))

(def-process-module pr2-manipulation-process-module (desig)
  (collision-environment-set-laser-period)
  (apply #'call-action (reference desig)))

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
  (let ((obj (newest-valid-designator obj)))
    (multiple-value-bind (left-grasp-pose right-grasp-pose)
        (compute-both-arm-grasping-poses obj)
      (execute-both-arm-grasp left-grasp-pose right-grasp-pose)
      ;; TODO(Georg): make this depend on the execution result
      ;; update object designator, because it is now in the grippers
      ;; TODO(Georg): this constant is pot-specific -> move it somewhere else
      (update-picked-up-object-designator obj 'desig-props:both-grippers :left 0.0))))

(defun update-picked-up-object-designator (obj-desig gripper side height)
  "Function that creates and equates a new obj-designator to an object
that has been grasped. `gripper' shall either include the symbols
desig-props:gripper or desig-props:both-grippers to discriminate between
single and dual grasps. `Side' indicates with respect to which gripper
the new location designator shall be constructed. `height' is the
difference in z-coordinate of the grasping point of the object and
its' supporting plane."
  ;; get current pose of the object in map frame
  (let* ((obj-pose (cl-tf:transform-pose
                    *tf* :pose (obj-desig-location (current-desig obj-desig))
                         :target-frame "/map"))
         ;; build a new location designator for the object:
         ;; the transform will be in the wrist frame of the `side' gripper
         ;; thus it'll move with the gripper;
         ;; adding (in ,`gripper') indicates whether it has been grasped
         ;; with one or two grippers;
         ;; (orientation ...) is the intended put down orientation of the object;
         (new-loc-desig (make-designator
                         'location
                         `((in ,gripper)
                           (side ,side)
                           (pose ,(tf:copy-pose-stamped
                                   (cl-tf:transform-pose
                                    *tf* :pose obj-pose
                                         :target-frame (ecase side
                                                         (:right "/r_wrist_roll_link")
                                                         (:left "/l_wrist_roll_link")))
                                   :stamp 0.0))
                           (height ,height)
                           (orientation ,(cl-transforms:orientation obj-pose))))))
    ;; build and equate new object designator using the new location designator
    ;; NOTE: this usage of make-designator does it both in one line
    (make-designator
     'object
     `((at ,new-loc-desig) . ,(remove 'at (description obj-desig) :key #'car))
     obj-desig)))

(defun compute-both-arm-grasping-poses (obj)
  "Computes and returns the two grasping poses for an object `obj'
that has to be grasped with two grippers."
  (let* ((object-type (desig-prop-value obj 'type))
         (object-pose (desig:designator-pose obj))
         (object-transform (cl-transforms:pose->transform object-pose)))
    (cond ((eq object-type 'desig-props:pot)
           (let* (;; get grasping poses for the handles
                  (left-grasp-pose (cl-transforms:transform-pose
                                    object-transform
                                    *pot-relative-left-handle-transform*))
                  (right-grasp-pose (cl-transforms:transform-pose
                                     object-transform
                                    *pot-relative-right-handle-transform*))
                  ;; make 'em stamped poses
                  (left-grasp-stamped-pose (cl-tf:make-pose-stamped
                                            (cl-tf:frame-id object-pose)
                                            (cl-tf:stamp object-pose)
                                            (cl-transforms:origin left-grasp-pose)
                                            (cl-transforms:orientation left-grasp-pose)))
                  (right-grasp-stamped-pose (cl-tf:make-pose-stamped
                                             (cl-tf:frame-id object-pose)
                                             (cl-tf:stamp object-pose)
                                             (cl-transforms:origin right-grasp-pose)
                                             (cl-transforms:orientation right-grasp-pose))))
             ;; return the grasping poses
             (values left-grasp-stamped-pose right-grasp-stamped-pose)))
          ;; TODO(Georg): change these strings into symbols from the designator package
          ((equal object-type "BigPlate")
           ;; TODO(Georg): replace these identities with the actual values
           (let ((left-grasp-pose (cl-transforms:make-identity-pose))
                 (right-grasp-pose (cl-transforms:make-identity-pose)))
             (values left-grasp-pose right-grasp-pose)))
          (t (error 'manipulation-failed
                    :format-control "Invalid objects for dual-arm manipulation specified.")))))

(defun calc-seed-state-elbow-up (side)
  ;; get names, current-values, lower-limits, and upper-limits for arm
  ;; build joint-state msg with current values for arm
  ;; set second joint to maximum, i.e. elbow up
  ;; return msg
  (multiple-value-bind (names current lower-limits upper-limits)
      (get-arm-state side)
    (let ((desired-positions
            (map 'vector #'identity
                 (loop for name across names collecting
                       (cond ((and (equal (aref names 2) name)
                                   (eql side :right))
                              (+ (/ PI 4) (gethash name lower-limits)))
                             ((and (equal (aref names 2) name)
                                   (eql side :left))
                              (- (gethash name upper-limits) (/ PI 4)))
                             ;; ((equal (aref names 1) name)
                             ;;  (gethash name lower-limits))
                             (t (gethash name current)))))))
      (roslisp:make-msg
       "sensor_msgs/JointState"
       (stamp header) 0
       name names
       position desired-positions
       velocity (make-array (length names)
                            :element-type 'float
                            :initial-element 0.0)
       effort (make-array (length names)
                          :element-type 'float
                          :initial-element 0.0)))))

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
