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
(defparameter *grasp-distance* 0.00
  "Tool length to calculate the pre-grasp pose, i.e. the pose at which
  the gripper is closed.")
(defparameter *pre-put-down-distance* 0.07
  "Distance above the goal before putting down the object")

(defparameter *max-graspable-size* (cl-transforms:make-3d-vector 0.15 0.15 0.30))

(defvar *open-container-action* nil)
(defvar *close-container-action* nil)

(defvar *gripper-action-left* nil)
(defvar *gripper-action-right* nil)
(defvar *gripper-grab-action-left* nil)
(defvar *gripper-grab-action-right* nil)

(defvar *trajectory-action-left* nil)
(defvar *trajectory-action-right* nil)
(defvar *trajectory-action-torso* nil)
(defvar *move-arm-right* nil)
(defvar *move-arm-left* nil)

(defvar *joint-state-sub* nil)

(defvar *open-trajectories* (tg:make-weak-hash-table :weakness :key))

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

(defun init-pr2-manipulation-process-module ()
  (setf *open-container-action* (actionlib:make-action-client
                                 "/open_container_action"
                                 "ias_drawer_actions/OpenContainerAction"))
  (setf *close-container-action* (actionlib:make-action-client
                                  "/close_container_action"
                                  "ias_drawer_actions/CloseContainerAction"))
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
  (let ((handle-pose (designator-pose handle)))
    (open-drawer handle-pose side))
  (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed)))

(def-action-handler container-closed (action)
  (execute-goal *close-container-action* action)
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
  (let* ((wrist-transform (tf:lookup-transform
                           *tf*
                           :time 0
                           :source-frame (ecase side
                                           (:right "r_wrist_roll_link")
                                           (:left "l_wrist_roll_link"))
                           :target-frame "/base_footprint"))
         (lift-pose (tf:make-pose-stamped
                     (tf:frame-id wrist-transform) (tf:stamp wrist-transform)
                     (cl-transforms:v+ (cl-transforms:translation wrist-transform)
                                       (cl-transforms:make-3d-vector 0 0 distance))
                     (cl-transforms:rotation wrist-transform))))
    (execute-arm-trajectory side (ik->trajectory (lazy-car (get-ik side lift-pose))))
    (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))))

(def-action-handler grasp (obj side obstacles)
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
                                  (calculate-grasp-pose
                                   obj
                                   :tool (calculate-tool-pose
                                          grasp
                                          *grasp-approach-distance*)))
                                (grasp-pose
                                  (calculate-grasp-pose
                                   obj
                                   :tool (calculate-tool-pose
                                          grasp
                                          *grasp-distance*))))
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
      ;; (compliant-close-girpper side)
      (close-gripper side 50.0)
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
                                      (:left "l_gripper_r_finger_tip_link"))))
    (assert-occasion `(object-in-hand ,obj ,side))))

(def-action-handler put-down (obj location side obstacles)
  (roslisp:ros-info (pr2-manip process-module) "Putting down object")
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
                  (:left *trajectory-action-left*)
                  (:right *trajectory-action-right*))))
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

(defun compliant-close-girpper (side)
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

(defun close-gripper (side &optional (max-effort 100.0))
  (let ((client (ecase side
                  (:right *gripper-action-right*)
                  (:left *gripper-action-left*))))
    (actionlib:send-goal-and-wait
     client (actionlib:make-action-goal client
              (position command) 0.0
              (max_effort command) max-effort)
     :result-timeout 1.0)))

(defun open-gripper (side &optional (max-effort 100.0))
  (let ((client (ecase side
                  (:right *gripper-action-right*)
                  (:left *gripper-action-left*))))
    (prog1
        (actionlib:send-goal-and-wait
         client (actionlib:make-action-goal client
                  (position command) 0.085
                  (max_effort command) max-effort)
         :result-timeout 1.0)
      (let ((obj (var-value
                  '?obj
                  (lazy-car
                   (rete-holds `(object-in-hand ?obj ,side))))))
        (unless (is-var obj)
          (retract-occasion `(object-in-hand ,obj ,side)))))))

(defun store-open-trajectory (obj open-result)
  (declare (type object-designator obj)
           (type ias_drawer_actions-msg:opencontainergoal open-result))
  (roslisp:with-fields (trajectory) open-result
    (setf (gethash obj *open-trajectories*) trajectory)))

(defun get-open-trajectory (obj)
  (declare (type object-designator obj))
  (gethash obj *open-trajectories*))

(def-process-module pr2-manipulation-process-module (desig)
  (collision-environment-set-laser-period)
  (apply #'call-action (reference desig)))

(defun open-drawer (pose side)
  (let* ((pre-grasp-pose (tf:pose->pose-stamped
                          (tf:frame-id pose) (tf:stamp pose)
                          (cl-transforms:transform-pose
                           (cl-transforms:make-transform
                            (cl-transforms:make-3d-vector
                             -0.25 0.0 0.0)
                            (cl-transforms:make-identity-rotation))
                           pose)))
         (grasp-pose (tf:pose->pose-stamped
                      (tf:frame-id pose) (tf:stamp pose)
                      (cl-transforms:transform-pose
                       (cl-transforms:make-transform
                        (cl-transforms:make-3d-vector
                         -0.20 0.0 0.0)
                        (cl-transforms:make-identity-rotation))
                       pose)))
         (open-pose (tf:pose->pose-stamped
                     (tf:frame-id pose) (tf:stamp pose)
                     (cl-transforms:transform-pose
                      (cl-transforms:make-transform
                       (cl-transforms:make-3d-vector
                        -0.35 0.0 0.0)
                       (cl-transforms:make-identity-rotation))
                      pose)))
         (pre-grasp-ik (lazy-car (get-ik side pre-grasp-pose)))
         (grasp-ik (lazy-car (get-ik side grasp-pose)))
         (open-ik (lazy-car (get-ik side open-pose))))
    (unless pre-grasp-ik
      (error 'manipulation-pose-unreachable
             :format-control "Pre-grasp pose for handle not reachable."))
    (unless grasp-ik
      (error 'manipulation-pose-unreachable
             :format-control "Pre-grasp pose for handle not reachable."))
    (unless open-ik
      (error 'manipulation-pose-unreachable
             :format-control "Pre-grasp pose for handle not reachable."))
    (open-gripper side)
    (execute-arm-trajectory side (ik->trajectory pre-grasp-ik))
    (execute-arm-trajectory side (ik->trajectory grasp-ik))
    (close-gripper side)
    (execute-arm-trajectory side (ik->trajectory open-ik))
    (open-gripper side)))

