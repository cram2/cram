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
                                   0.09855628758792889d0 -0.48 0.8283385079087044d0)
                                  (cl-transforms:make-identity-rotation)))
(defparameter *carry-pose-left* (tf:make-pose-stamped
                                 "/base_footprint" 0.0
                                 (cl-transforms:make-3d-vector
                                  0.09855628758792889d0 0.48 0.8283385079087044d0)
                                 (cl-transforms:make-identity-rotation)))

(defparameter *top-grasp* (cl-transforms:euler->quaternion :ay (/ pi 2)))
(defparameter *front-grasp* (cl-transforms:make-identity-rotation))
(defparameter *left-grasp* (cl-transforms:euler->quaternion :az (/ pi -2)))
(defparameter *right-grasp* (cl-transforms:euler->quaternion :az (/ pi 2)))

(defun init-pr2-manipulation-process-module ()
  (setf *open-container-action* (actionlib:make-action-client
                              "/open_container_action"
                              "ias_drawer_executive/OpenContainerAction"))
  (setf *close-container-action* (actionlib:make-action-client
                               "/close_container_action"
                               "ias_drawer_executive/CloseContainerAction"))
  (setf *gripper-action-left* (actionlib:make-action-client
                               "/l_gripper_sensor_controller/gripper_action"
                               "pr2_controllers_msgs/Pr2GripperCommandAction"))
  (setf *gripper-action-right* (actionlib:make-action-client
                                "/r_gripper_sensor_controller/gripper_action"
                                "pr2_controllers_msgs/Pr2GripperCommandAction"))
  (setf *gripper-grab-action-left* (actionlib:make-action-client
                                    "/l_gripper_sensor_controller/grab"
                                    "pr2_gripper_sensor_msgs/PR2GripperGrabAction"))
  (setf *gripper-grab-action-right* (actionlib:make-action-client
                                     "/r_gripper_sensor_controller/grab"
                                     "pr2_gripper_sensor_msgs/PR2GripperGrabAction"))
  (setf *trajectory-action-left* (actionlib:make-action-client
                                  "/l_arm_controller/joint_trajectory_action"
                                  "pr2_controllers_msgs/JointTrajectoryAction"))
  (setf *trajectory-action-right* (actionlib:make-action-client
                                  "/r_arm_controller/joint_trajectory_action"
                                  "pr2_controllers_msgs/JointTrajectoryAction"))
  (setf *trajectory-action-torso* (actionlib:make-action-client
                                  "/torso_controller/joint_trajectory_action"
                                  "pr2_controllers_msgs/JointTrajectoryAction"))
  (setf *move-arm-right* (actionlib:make-action-client
                          "/move_right_arm"
                          "move_arm_msgs/MoveArmAction"))
  (setf *move-arm-left* (actionlib:make-action-client
                         "/move_left_arm"
                         "move_arm_msgs/MoveArmAction"))  
  (setf *joint-state-sub* (roslisp:subscribe
                           "/joint_states" "sensor_msgs/JointState"
                           (lambda (msg)
                             (setf (cpl:value *joint-state*)
                                   msg)))))

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

(def-action-handler container-opened (action obj)
  (store-open-trajectory
   obj (execute-goal *open-container-action* action)))

(def-action-handler container-closed (action)
  (execute-goal *close-container-action* action))

(def-action-handler park (obj side)
  (roslisp:ros-info (pr2-manip process-module) "Park arms ~a ~a"
                    obj side)
  (let ((orientation (calculate-carry-orientation obj))
        (carry-pose (ecase side
                      (:right *carry-pose-right*)
                      (:left *carry-pose-left*))))
    (if orientation
        (execute-move-arm side (tf:copy-pose-stamped carry-pose :orientation orientation))
        (execute-move-arm side carry-pose))))

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
    (execute-arm-trajectory side (ik->trajectory (lazy-car (get-ik side lift-pose))))))

(def-action-handler grasp (obj side)
  (roslisp:ros-info (pr2-manip process-module) "Opening gripper")
  (open-gripper side)
  (roslisp:ros-info (pr2-manip process-module) "Clearing collision map")
  (clear-collision-objects)
  (roslisp:ros-info (pr2-manip process-module) "Adding object as collision object")
  (register-collision-object obj)
  (roslisp:ros-info (pr2-manip process-module) "Registering semantic map objects")
  (roslisp:ros-info (pr2-manip process-module) "Calling grasp planner")
  (sem-map-coll-env:publish-semantic-map-collision-objects)
  (let ((grasp-poses
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
                                           (list *left-grasp*))))
                                     (lazy-car
                                      (get-ik
                                       side pre-grasp-pose))
                                     (lazy-car
                                      (get-ik
                                       side grasp-pose)))
                            (roslisp:ros-info (pr2-manip process-module) "Found valid grasp ~a ~a"
                                              pre-grasp-pose grasp-pose)
                            `((,pre-grasp-pose ,grasp-pose)))))
                      (prog2
                          (roslisp:ros-info (pr2-manip process-module) "Planning grasp")
                          (get-point-cluster-grasps side obj)
                        (roslisp:ros-info (pr2-manip process-module) "Grasp planning finished")))))
    (unless grasp-poses
      (error 'manipulation-pose-unreachable
             :format-control "No valid grasp pose found"))
    (roslisp:ros-info (pr2-manip process-module) "Executing move-arm")
    (or
     (lazy-car (lazy-mapcar
                (lambda (grasp-pose)
                  (destructuring-bind (pre-grasp grasp) grasp-pose
                    (execute-move-arm side pre-grasp :ompl)
                    (execute-arm-trajectory side (ik->trajectory (lazy-car (get-ik side grasp))))))
                grasp-poses))
     (cpl-impl:fail 'manipulation-pose-unreachable))
    (roslisp:ros-info (pr2-manip process-module) "Closing gripper")
    (compliant-close-girpper side)
    ;; TODO: Check if gripper is not completely closed to make sure that we are holding the object
    (roslisp:ros-info (pr2-manip process-module) "Attaching object to gripper")
    (attach-collision-object side obj)
    (assert-occasion `(object-in-hand ,obj ,side))))

(def-action-handler put-down (obj location side)
  (roslisp:ros-info (pr2-manip process-module) "Putting down object")
  (assert (and (rete-holds `(object-in-hand ,obj ,side))) ()
          "Object ~a needs to be in the gripper" obj)
  (let* ((put-down-pose (calculate-put-down-pose obj location))
         (pre-put-down-pose (tf:copy-pose-stamped
                             put-down-pose
                             :origin (cl-transforms:v+
                                      (cl-transforms:origin put-down-pose)
                                      (cl-transforms:make-3d-vector 0 0 0.05)))))
    (execute-move-arm side pre-put-down-pose :ompl)
    (execute-arm-trajectory side (ik->trajectory (lazy-car (get-ik side put-down-pose))))
    (open-gripper side)
    (execute-arm-trajectory side (ik->trajectory (lazy-car (get-ik side pre-put-down-pose))))))

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

(defun execute-move-arm (side pose &optional (planner :ompl))
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
             (num_planning_attempts motion_plan_request) 10
             (planner_id motion_plan_request) ""
             (allowed_planning_time motion_plan_request) 10.0
        
             (position_constraints goal_constraints motion_plan_request)
             (vector
              (roslisp:make-msg
               "motion_planning_msgs/PositionConstraint"
               header header
               link_name (ecase side
                           (:left "l_wrist_roll_link")
                           (:right "r_wrist_roll_link"))
               position position
               (type constraint_region_shape) (roslisp-msg-protocol:symbol-code
                                               'geometric_shapes_msgs-msg:shape
                                               :box)
               (dimensions constraint_region_shape) #(0.01 0.01 0.01)
               (w constraint_region_orientation) 1.0
               weight 1.0))
        
             (orientation_constraints goal_constraints motion_plan_request)
             (vector
              (roslisp:make-msg
               "motion_planning_msgs/OrientationConstraint"
               header header
               link_name (ecase side
                           (:left "l_wrist_roll_link")
                           (:right "r_wrist_roll_link"))
               orientation orientation
               absolute_roll_tolerance 0.01
               absolute_pitch_tolerance 0.01
               absolute_yaw_tolerance 0.01
               weight 1.0))
             
             (collision_operations ordered_collision_operations motion_plan_request)
             (vector
              (roslisp:make-msg
               "motion_planning_msgs/CollisionOperation"
               object1 "gripper"
               object2 "attached"
               penetration_distance 0.1
               operation 0)))
           :result-timeout 1.0)
        (case val
          (1 t)
          (-31 (error 'move-arm-no-ik-solution))
          (-33 (error 'move-arm-ik-link-in-collision))
          (t (error 'manipulation-failed)))))))

(defun compliant-close-girpper (side)
  (roslisp:call-service
   (ecase side
    (:right "/r_reactive_grasp/compliant_close")
    (:left "/l_reactive_grasp/compliant_close"))
   'std_srvs-srv:Empty)
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
          (detach-collision-object side obj)
          (retract-occasion `(object-in-hand obj ,side)))))))

(defun store-open-trajectory (obj open-result)
  (declare (type object-designator obj)
           (type ias_drawer_executive-msg:opencontainergoal open-result))
  (roslisp:with-fields (trajectory) open-result
    (setf (gethash obj *open-trajectories*) trajectory)))

(defun get-open-trajectory (obj)
  (declare (type object-designator obj))
  (gethash obj *open-trajectories*))

(defun calculate-carry-orientation (obj)
  (when obj
    (let ((location-desig (desig-prop-value (current-desig obj) 'at)))
      (desig-prop-value location-desig 'orientation))))

(def-process-module pr2-manipulation-process-module (desig)
  (apply #'call-action (reference desig)))
