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

(defparameter *wrench-too-high-limit* 3.0 "In N/m.")

(defvar *wrench-state-sub* nil
  "Subscriber for robot's 6dof force-torque wrist sensor.")

(defvar *wrench-state-fluent* (cpl:make-fluent :name :wrench-state)
  "ROS message containing robot's left gripper state ROS message.")

(defun init-wrench-state-sub ()
  "Initializes *wrench-state-sub*"
  (flet ((wrench-state-sub-cb (wrench-state-msg)
           (setf (cpl:value *wrench-state-fluent*) wrench-state-msg)))
    (setf *wrench-state-sub*
          (roslisp:subscribe "left_arm_kms40/wrench_zeroed"
                             "geometry_msgs/WrenchStamped"
                             #'wrench-state-sub-cb))))

(defun destroy-wrench-state-sub ()
  (setf *wrench-state-sub* nil))

(roslisp-utilities:register-ros-init-function init-wrench-state-sub)
(roslisp-utilities:register-ros-cleanup-function destroy-wrench-state-sub)

(defun zero-wrench-sensor ()
  (loop until (roslisp:wait-for-service "ft_cleaner/update_offset" 5.0)
        do (roslisp:ros-info (force-torque-sensor zero-sensor) "Waiting for zeroing service..."))
  (roslisp:call-service "ft_cleaner/update_offset" 'std_srvs-srv:trigger))
