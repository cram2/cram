;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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
;;;       Technische Universitaet Muenchen nor the names of its contributors 
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

(define-condition add-tf-relay-error (simple-error) ())
(define-condition remove-tf-relay-error (simple-error) ())

(defvar *add-tf-relay-action* nil)
(defvar *remove-tf-relay-action* nil)

(defun init-feature-controllers () 
  (setf *add-tf-relay-action* (actionlib:make-action-client
                               "/tf_relay/addRelay"
                               "realtime_tf_relay_msgs/AddTfRelayAction"))
  (setf *remove-tf-relay-action* (actionlib:make-action-client
                                  "/tf_relay/removeRelay"
                                  "realtime_tf_relay_msgs/RemoveTfRelayAction")))

(register-ros-init-function init-feature-controllers)

(defun add-tf-relay (&key parent-frame child-frame topic-name (frequency 50.0))
  (let ((goal (actionlib:make-action-goal
                  *add-tf-relay-action*
                (child_frame relay) child-frame
                (parent_frame relay) parent-frame
                (topic_name relay) topic-name
                (frequency relay) frequency)))
    (multiple-value-bind (msg status)
        (actionlib:call-goal *add-tf-relay-action* goal)
      (declare (ignore msg))
      (unless (eq status :succeeded)
        (error 'add-tf-relay-error
               :format-control "Adding of tf-relay failed. Parent-frame: ~a, child-frame: ~a, topic-name: ~a"
               :format-arguments (list parent-frame child-frame topic-name))))))

(defun remove-tf-relay (&key (parent-frame "ignore-frame") (child-frame "ignored-frame")
                             topic-name (frequency 50.0))
  (let ((goal (actionlib:make-action-goal
                  *remove-tf-relay-action*
                (child_frame relay) child-frame
                (parent_frame relay) parent-frame
                (topic_name relay) topic-name
                (frequency relay) frequency)))
    (multiple-value-bind (msg status)
        (actionlib:call-goal *remove-tf-relay-action* goal)
      (declare (ignore msg))
      (unless (eq status :succeeded)
        (error 'remove-tf-relay-error
               :format-control "Removing of tf-relay failed. Topic-name: ~a"
               :format-arguments (list topic-name))))))

(defun set-up-tf-relays (&key (namespace "/left_arm_feature_controller")
                              (map-frame "/base_link")
                              (robot-base-frame "/base_link")
                              (arm-base-frame "/torso_lift_link")
                              (arm-tool-frame "/l_gripper_tool_frame")
                              (tool-frame "/l_gripper_tool_frame")
                              (object-frame "/torso_lift_link"))
  (declare (type string namespace)
           (type string map-frame)
           (type string robot-base-frame)
           (type string arm-base-frame)
           (type string arm-tool-frame)
           (type string tool-frame)
           (type string object-frame))
  ;; offset between the two links that were used to calculate the robot kinematics
  ;; NOTE: needs to be equal to 'base_frame' and 'tool_frame' in the feature-controller launch-file 
  (add-tf-relay :parent-frame robot-base-frame
                :child-frame arm-base-frame
                :topic-name (concatenate 'string namespace "/arm_offset"))
  ;; offset from 'where-objects are denoted in' to robot-base-frame
  ;; hook to fill in localization and map-resolved poses...
  (add-tf-relay :parent-frame map-frame
                :child-frame robot-base-frame
                :topic-name (concatenate 'string namespace "/base_pose"))
  ;; offset from end of arm to tool that is used
  ;; hook to fill in tool calibration...
  (add-tf-relay :parent-frame arm-tool-frame
                :child-frame tool-frame
                :topic-name (concatenate 'string namespace "/tool_offset"))
  ;; offset from 'where-objects are denoted in' to 'where object-features are defined in'
  ;; hook for object-perception...
  (add-tf-relay :parent-frame map-frame
                :child-frame object-frame
                :topic-name (concatenate 'string namespace "/object_offset")))

(defun shutdown-tf-relays (&key (namespace "/left_arm_feature_controller"))
  (declare (type string namespace))
  (remove-tf-relay :topic-name (concatenate 'string namespace "/arm_offset"))
  (remove-tf-relay :topic-name (concatenate 'string namespace "/base_pose"))
  (remove-tf-relay :topic-name (concatenate 'string namespace "/tool_offset"))
  (remove-tf-relay :topic-name (concatenate 'string namespace "/object_offset")))

;;; TODO(Georg): add functionality to 'blindly' shutdown all relays