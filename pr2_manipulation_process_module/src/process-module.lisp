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
  (prog1 (call-next-method))
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
