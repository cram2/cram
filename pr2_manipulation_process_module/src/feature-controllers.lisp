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
    (actionlib:call-goal *add-tf-relay-action* goal)))

(defun remove-tf-relay (&key (parent-frame "ignore-frame") (child-frame "ignored-frame")
                             topic-name (frequency 50.0))
  (let ((goal (actionlib:make-action-goal
                  *remove-tf-relay-action*
                (child_frame relay) child-frame
                (parent_frame relay) parent-frame
                (topic_name relay) topic-name
                (frequency relay) frequency)))
    (actionlib:call-goal *remove-tf-relay-action* goal)))

(defun set-up-tf-relays (&key (namespace "/left_arm_feature_controller"))
  (declare (type string namespace))
  ;; offset from /base_link to /torso_lift_link 
  ;; will always be needed like this
  (add-tf-relay :parent-frame "/base_link" ; always like this
                :child-frame "/torso_lift_link" ; always like this
                :topic-name (concatenate 'string namespace "/arm_offset")) ; always like this
  ;; offset from 'where-objects are denoted in' to /base_link
  ;; hook to fill in localization and map-resolved poses...
  (add-tf-relay :parent-frame "/base_link" ; variable
                :child-frame "/base_link" ; always like this
                :topic-name (concatenate 'string namespace "/base_pose")) ; always like this
  ;; offset from /l_gripper_tool_frame to 'where my tool is'
  ;; hook to fill in  object calibration...
  (add-tf-relay :parent-frame "/l_gripper_tool_frame" ; always like this (IF w/o nullspace of arm)
                :child-frame "/l_gripper_tool_frame" ; variable
                :topic-name (concatenate 'string namespace "/tool_offset")) ; always like this
  ;; offset from 'where-objects are denoted in' to 'where object-features are defined in'
  ;; hook for object-perception...
  (add-tf-relay :parent-frame "/base_link" ; variable
                :child-frame "/torso_lift_link" ; variable
                :topic-name (concatenate 'string namespace "/object_offset"))) ;always like this

(defun shutdown-tf-relays (&key (namespace "/left_arm_feature_controller"))
  (declare (type string namespace))
  (remove-tf-relay :topic-name (concatenate 'string namespace "/arm_offset"))
  (remove-tf-relay :topic-name (concatenate 'string namespace "/base_pose"))
  (remove-tf-relay :topic-name (concatenate 'string namespace "/tool_offset"))
  (remove-tf-relay :topic-name (concatenate 'string namespace "/object_offset")))