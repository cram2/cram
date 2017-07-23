;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :boxy-ll)

(defvar *force-torque-state-sub* nil
  "Subscriber for robot's 6dof force-torque wrist sensor.")

(defvar *force-torque-state-fluent* (cpl:make-fluent :name :force-torque-state)
  "ROS message containing robot's left gripper state ROS message.")

(defun init-force-torque-state-sub ()
  "Initializes *left-gripper-state-sub*"
  (flet ((force-torque-state-sub-cb (force-torque-state-msg)
           (setf (cpl:value *force-torque-state-msg*) force-torque-state-msg)))
    (setf *force-torque-state-sub*
          (roslisp:subscribe "namespace/topic"
                             "iai_msgs/MessageType"
                             #'force-torque-state-sub-cb))))

(defun destroy-force-torque-state-sub ()
  (setf *force-torque-state-sub* nil))

(roslisp-utilities:register-ros-init-function init-force-torque-state-sub)
(roslisp-utilities:register-ros-cleanup-function destroy-force-torque-state-sub)

