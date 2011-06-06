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

(defvar *open-handle-action* nil)
(defvar *close-handle-action* nil)
;; (defvar *pick-bottle-action* nil)
;; (defvar *pick-plate-action* nil)
(defvar *open-fridge-action* nil)
(defvar *close-fridge-action* nil)
(defvar *take-bottle-action* nil)
(defvar *serve-bottle-action* nil)
(defvar *take-plate-action* nil)
(defvar *take-plate-from-island-action* nil)
(defvar *serve-plate-to-island-action* nil)
(defvar *serve-plate-to-table-action* nil)

(defvar *trajectory-action-left* nil)
(defvar *trajectory-action-right* nil)
(defvar *trajectory-action-torso* nil)
(defvar *move-arm-right* nil)
(defvar *move-arm-left* nil)

(defvar *joint-state-sub* nil)


(defun init-pr2-manipulation-process-module ()
  ;; (setf *open-handle-action* (actionlib:make-action-client
  ;;                             "/operate_handle_action"
  ;;                             "ias_drawer_executive/OperateHandleAction"))
  ;; (setf *close-handle-action* (actionlib:make-action-client
  ;;                              "/close_handle_action"
  ;;                              "ias_drawer_executive/CloseHandleAction"))
  ;; (setf *pick-bottle-action* (actionlib:make-action-client
  ;;                             "/pick_bottle_action"
  ;;                             "ias_drawer_executive/PickBottleAction"))
  ;; (setf *pick-plate-action* (actionlib:make-action-client
  ;;                            "/pick_plate_action"
  ;;                            "ias_drawer_executive/PickPlateAction"))
  ;;; Demo actions
  (setf *open-fridge-action* (actionlib:make-action-client
                              "/open_fridge"
                              "ias_drawer_executive/GenericAction"))
  (setf *close-fridge-action* (actionlib:make-action-client
                               "/close_fridge"
                               "ias_drawer_executive/GenericAction"))
  (setf *open-handle-action* (actionlib:make-action-client
                              "/open_drawer"
                              "ias_drawer_executive/GenericAction"))
  (setf *close-handle-action* (actionlib:make-action-client
                               "/close_drawer"
                               "ias_drawer_executive/GenericAction"))
  (setf *take-bottle-action* (actionlib:make-action-client
                               "/take_bottle"
                               "ias_drawer_executive/GenericAction"))
  (setf *serve-bottle-action* (actionlib:make-action-client
                              "/serve_bottle"
                              "ias_drawer_executive/GenericAction"))
  (setf *take-plate-action* (actionlib:make-action-client
                               "/take_drawer"
                               "ias_drawer_executive/GenericAction"))
  (setf *take-plate-from-island-action* (actionlib:make-action-client
                               "/take_plate_from_island"
                               "ias_drawer_executive/GenericAction"))  
  (setf *serve-plate-to-island-action* (actionlib:make-action-client
                              "/serve_plate_to_island"
                              "ias_drawer_executive/GenericAction"))
  (setf *serve-plate-to-table-action* (actionlib:make-action-client
                              "/serve_to_table"
                              "ias_drawer_executive/GenericAction"))
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

(defgeneric call-action (action goal params))

;; (defmethod call-action ((action-sym (eql 'ik) pose params))
;;   (destructuring-bind (obj side) params
;;     (let ((ik-solution )))))

(defmethod call-action ((action-sym (eql 'fridge-opened)) goal params)
  (let ((result (execute-goal *open-fridge-action* goal)))
    (destructuring-bind (obj side) params
      (roslisp:with-fields (handle) result
        (retract-occasion `(object-closed ?obj))
        (assert-occasion `(object-opened ,obj ,side))
        (assert-occasion `(fridge-open-handle ,obj ,handle))))))

(defmethod call-action ((action-sym (eql 'fridge-closed)) goal params)
  (execute-goal *close-fridge-action* goal)
  (destructuring-bind (obj side) params
    (retract-occasion `(object-opened ,obj ,side))
    (retract-occasion `(fridge-open-handle ,obj ?_))
    (assert-occasion `(object-closed ,obj))))

(defmethod call-action ((action-sym (eql 'drawer-opened)) goal params)
  (let ((result (execute-goal *open-handle-action* goal)))
    (destructuring-bind (obj side) params
      (roslisp:with-fields (handle) result
        (retract-occasion `(object-closed ?obj))
        (assert-occasion `(object-opened ,obj ,side))
        (assert-occasion `(drawer-open-handle ,obj ,handle))))))

(defmethod call-action ((action-sym (eql 'drawer-closed)) goal params)
  (execute-goal *close-handle-action* goal)
  (destructuring-bind (obj side) params
    (retract-occasion `(object-opened ,obj ,side))
    (retract-occasion `(drawer-open-handle ,obj ?_))
    (assert-occasion `(object-closed ,obj))))

(defmethod call-action ((action-sym (eql 'plate-grasped-drawer)) goal params)
  (let ((result (execute-goal *take-plate-action* goal)))
    (destructuring-bind (obj) params
      (roslisp:with-fields (handle) result
        (assert-occasion `(plate-grasped-handle ,obj ,handle))))))

(defmethod call-action ((action-sym (eql 'bottle-grasped)) goal params)
  (let ((result (execute-goal *take-bottle-action* goal)))
    (destructuring-bind (obj) params
      (roslisp:with-fields (handle) result
        (assert-occasion `(bottle-grasped-handle ,obj ,handle))))))

(defmethod call-action ((action-sym (eql 'plate-put-down-island)) goal params)
  (execute-goal *serve-plate-to-island-action* goal)
  (destructuring-bind (obj) params
    (retract-occasion `(plate-grasped-handle ,obj ?_))))

(defmethod call-action ((action-sym (eql 'bottle-put-down-island)) goal params)
  (execute-goal *serve-bottle-action* goal)
  (destructuring-bind (obj) params
    (retract-occasion `(bottle-grasped-handle ,obj ?_))))

(defmethod call-action ((action-sym (eql 'plate-grasped-island)) goal params)
  (execute-goal *serve-plate-to-island-action* goal))

(defmethod call-action ((action-sym (eql 'plate-put-down-table)) goal params)
  (execute-goal *serve-plate-to-table-action* goal))

(defmethod call-action ((action-sym t) goal params)
  (declare (ignore goal params))
  (roslisp:ros-info (pr2-manip process-module)
                    "Unimplemented operation `~a'. Doing nothing."
                    action-sym)
  (sleep 0.5))

(defmethod call-action :around (action-sym goal params)
  (roslisp:ros-info (pr2-manip process-module) "Executing manipulation action ~a ~a." action-sym goal)
  (call-next-method)
  (roslisp:ros-info (pr2-manip process-module) "Manipulation action done."))

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

(defun execute-move-arm (side pose)
  (let ((action (ecase side
                  (:left *move-arm-left*)
                  (:right *move-arm-right*)))
        (pose-msg (tf:pose-stamped->msg pose)))
    (roslisp:with-fields (header
                          (position (position pose))
                          (orientation (orientation pose)))
        pose-msg
      (actionlib:call-goal
       action  
       (actionlib:make-action-goal
        action
        planner_service_name "/ompl_planning/plan_kinematic_path"
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
          weight 1.0)))))))

(def-process-module pr2-manipulation-process-module (desig)
  (apply #'call-action (reference desig)))
