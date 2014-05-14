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

(in-package :pr2-manipulation-process-module)

(defclass grasp-assignment ()
  ((pose :accessor pose :initform nil :initarg :pose)
   (side :accessor side :initform nil :initarg :side)
   (close-radius :accessor close-radius :initform nil :initarg :close-radius)
   (handle-pair :accessor handle-pair :initform nil :initarg :handle-pair)
   (ik-cost :accessor ik-cost :initform nil :initarg :ik-cost)))

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

(defvar *joint-state-sub* nil)

(defvar *left-safe-pose* (tf:make-pose-stamped
                          "base_link" (ros-time)
                          (tf:make-3d-vector 0.3 0.5 1.3)
                          (tf:euler->quaternion :ax pi)))
(defvar *right-safe-pose* (tf:make-pose-stamped
                           "base_link" (ros-time)
                           (tf:make-3d-vector 0.3 -0.5 1.3)
                           (tf:euler->quaternion :ax pi)))

(defun init-pr2-manipulation-process-module ()
  (setf *gripper-action-left*
        (actionlib:make-action-client
         "/l_gripper_controller/gripper_action"
         "pr2_controllers_msgs/Pr2GripperCommandAction"))
  (setf *gripper-action-right*
        (actionlib:make-action-client
         "/r_gripper_controller/gripper_action"
         "pr2_controllers_msgs/Pr2GripperCommandAction"))
  (setf *trajectory-action-torso*
        (actionlib:make-action-client
         "/torso_controller/joint_trajectory_action"
         "pr2_controllers_msgs/JointTrajectoryAction"))
  (setf *joint-state-sub*
        (roslisp:subscribe
         "/joint_states" "sensor_msgs/JointState"
         (lambda (msg)
           (setf *joint-state* msg))))
  (set-robot-planning-state))

(roslisp-utilities:register-ros-init-function
 init-pr2-manipulation-process-module)

(defun execute-goal (server goal)
  (multiple-value-bind (result status)
      (actionlib:call-goal server goal)
    (unless (eq status :succeeded)
      (cpl-impl:fail 'manipulation-failed
                     :format-control "Manipulation failed"))
    result))

(defun links-for-arm-side (side)
  (ecase side
    (:left (list "l_shoulder_pan_link"
                 "l_shoulder_lift_link"
                 "l_upper_arm_roll_link"
                 "l_upper_arm_link"
                 "l_elbow_flex_link"
                 "l_forearm_roll_link"
                 "l_forearm_link"
                 "l_wrist_flex_link"
                 "l_wrist_roll_link"
                 "l_gripper_led_frame"
                 "l_gripper_motor_accelerometer_link"
                 "l_gripper_tool_frame"
                 "l_gripper_r_finger_link"
                 "l_gripper_r_finger_tip_link"
                 "l_gripper_l_finger_tip_frame"
                 "l_gripper_l_finger_link"
                 "l_gripper_l_finger_tip_link"
                 "l_gripper_motor_slider_link"
                 "l_gripper_motor_screw_link"
                 "l_gripper_palm_link"
                 "l_force_torque_link"
                 "l_force_torque_adapter_link"))
    (:right (list "r_gripper_palm_link"
                  "r_shoulder_pan_link"
                  "r_shoulder_lift_link"
                  "r_upper_arm_roll_link"
                  "r_upper_arm_link"
                  "r_elbow_flex_link"
                  "r_forearm_roll_link"
                  "r_forearm_link"
                  "r_wrist_flex_link"
                  "r_wrist_roll_link"
                  "r_gripper_led_frame"
                  "r_gripper_motor_accelerometer_link"
                  "r_gripper_tool_frame"
                  "r_gripper_r_finger_link"
                  "r_gripper_r_finger_tip_link"
                  "r_gripper_l_finger_tip_frame"
                  "r_gripper_l_finger_link"
                  "r_gripper_l_finger_tip_link"
                  "r_gripper_motor_slider_link"
                  "r_gripper_motor_screw_link"))))

(define-hook on-prepare-move-arm (side pose-stamped planning-group ignore-collisions))
(define-hook on-finish-move-arm (log-id success))

(defun execute-move-arm-pose (side pose-stamped
                              &key allowed-collision-objects
                                ignore-collisions)
  (ros-info (pr2 manip-pm) "Executing arm movement")
  (let* ((allowed-collision-objects
           (cond (ignore-collisions
                  (append allowed-collision-objects
                          (links-for-arm-side side)))
                 (t allowed-collision-objects)))
         (link-name (ecase side
                      (:left "l_wrist_roll_link")
                      (:right "r_wrist_roll_link")))
         (planning-group (ecase side
                           (:left "left_arm")
                           (:right "right_arm"))))
    (let ((log-id (first (on-prepare-move-arm
                          link-name pose-stamped
                          planning-group ignore-collisions))))
      (cpl:with-failure-handling
          ((moveit:no-ik-solution (f)
             (declare (ignore f))
             (ros-error (move arm) "No IK solution found.")
             (on-finish-move-arm log-id nil)
             (error 'manipulation-pose-unreachable
                    :result (list side pose-stamped)))
           (moveit:planning-failed (f)
             (declare (ignore f))
             (ros-error (move arm) "Planning failed.")
             (on-finish-move-arm log-id nil)
             (error 'manipulation-pose-unreachable
                    :result (list side pose-stamped)))
           (moveit:goal-violates-path-constraints (f)
             (declare (ignore f))
             (ros-error (move arm) "Goal violates path constraints.")
             (on-finish-move-arm log-id nil)
             (error 'manipulation-pose-unreachable
                    :result (list side pose-stamped)))
           (moveit:invalid-goal-constraints (f)
             (declare (ignore f))
             (ros-error (move arm) "Invalid goal constraints.")
             (on-finish-move-arm log-id nil)
             (error 'manipulation-pose-unreachable
                    :result (list side pose-stamped)))
           (moveit:timed-out (f)
             (declare (ignore f))
             (ros-error (move arm) "Timeout.")
             (on-finish-move-arm log-id nil)
             (error 'manipulation-pose-unreachable
                    :result (list side pose-stamped)))
           (moveit:goal-in-collision (f)
             (declare (ignore f))
             (ros-error (move arm) "Goal in collision.")
             (on-finish-move-arm log-id nil)
             (error 'manipulation-pose-occupied
                    :result (list side pose-stamped))))
        (cond ((moveit:move-link-pose
                link-name
                planning-group pose-stamped
                :ignore-collisions ignore-collisions
                :allowed-collision-objects allowed-collision-objects
                :touch-links (links-for-arm-side side))
               (on-finish-move-arm log-id t)
               (plan-knowledge:on-event
                (make-instance 'plan-knowledge:robot-state-changed)))
              (t (on-finish-move-arm log-id nil)
                 (error 'manipulation-failed
                        :result (list side pose-stamped))))))))

(defun close-gripper (side &key (max-effort 100.0) (position 0.0))
  (let ((client (ecase side
                  (:right *gripper-action-right*)
                  (:left *gripper-action-left*))))
    (unwind-protect
         (actionlib:send-goal-and-wait
          client (actionlib:make-action-goal client
                   (position command) position
                   (max_effort command) max-effort)
          :result-timeout 1.0)
      (plan-knowledge:on-event (make-instance
                                'plan-knowledge:robot-state-changed)))))

(defun open-gripper (side &key (max-effort 100.0) (position 0.085))
  (let ((client (ecase side
                  (:right *gripper-action-right*)
                  (:left *gripper-action-left*))))
    (prog1
        (unwind-protect
             (actionlib:send-goal-and-wait
              client (actionlib:make-action-goal client
                       (position command) position
                       (max_effort command) max-effort)
              :result-timeout 1.0)
          (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed)))
      (with-vars-bound (?carried-object ?gripper-link)
          (lazy-car (prolog `(and (object-in-hand ?carried-object ,side)
                                  (cram-manipulation-knowledge:end-effector-link ,side ?gripper-link))))
        (unless (is-var ?carried-object)
          (plan-knowledge:on-event
           (make-instance 'object-detached
             :object ?carried-object
             :link ?gripper-link
             :side side)))))))

(defclass manipulated-perceived-object (desig:object-designator-data) ())

(defun update-object-designator-pose (object-designator new-object-pose)
  (equate object-designator
          (desig:make-effective-designator
           object-designator :data-object (make-instance 'manipulated-perceived-object
                                                         :pose new-object-pose))))

(def-process-module pr2-manipulation-process-module (desig)
  (collision-environment-set-laser-period)
  (apply #'call-action (reference desig)))

(defun update-grasped-object-designator (obj grippers &key new-properties)
  (let* ((target-frame (var-value '?target-frame
                                  (lazy-car
                                   (crs:prolog
                                    `(cram-pr2-knowledge::end-effector-link
                                      ,(car grippers)
                                      ?target-frame)))))
         (obj-pose-in-gripper (tf:pose->pose-stamped
                               target-frame
                               0.0
                               (cl-tf:transform-pose
                                *tf*
                                :pose (obj-desig-location
                                       (current-desig obj))
                                :target-frame target-frame)))
         (loc-desig-in-gripper (make-designator
                                'location
                                (append `((pose ,obj-pose-in-gripper)
                                          (in gripper))
                                        (mapcar (lambda (grip)
                                                  `(gripper ,grip))
                                                grippers)))))
    (make-designator
     'object
     (append `((at ,loc-desig-in-gripper) . ,(remove 'at (description obj) :key #'car))
             new-properties)
     obj)))

(defun update-picked-up-object-designator (obj-desig gripper side height)
  "Function that creates and equates a new obj-designator to an object
that has been grasped. `gripper' shall either include the symbols
GRIPPER or BOTH-GRIPPERS to discriminate between
single and dual grasps. `Side' indicates with respect to which gripper
the new location designator shall be constructed. `height' is the
difference in z-coordinate of the grasping point of the object and
its' supporting plane."
  (style-warn 'simple-style-warning
              :format-control "Use of deprecated form
              UPDATE-PICKED-UP-OBJECT-DESIGNATOR. Please use
              UPDATE-GRASPED-OBJECT-DESIGNATOR instead.")
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
    (cond ((eq object-type 'pot)
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

(defun set-robot-planning-state ()
  (let ((joint-names (list "r_shoulder_pan_joint"
                           "r_shoulder_lift_joint"
                           "r_upper_arm_roll_joint"
                           "r_elbow_flex_joint"
                           "r_forearm_roll_joint"
                           "r_wrist_flex_joint"
                           "r_wrist_roll_joint"
                           "l_shoulder_pan_joint"
                           "l_shoulder_lift_joint"
                           "l_upper_arm_roll_joint"
                           "l_elbow_flex_joint"
                           "l_forearm_roll_joint"
                           "l_wrist_flex_joint"
                           "l_wrist_roll_joint"
                           "torso_lift_joint")))
    (moveit::copy-physical-joint-states joint-names)))
