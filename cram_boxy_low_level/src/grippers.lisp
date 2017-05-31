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

(defvar *gripper-publishers* '(:left nil :right nil)
  "ROS publisher for Boxy gripper driver on goal_position message.")

(defun init-gripper-position-publishers ()
  (setf (getf *neck-configuration-publisher* :left)
        (roslisp:advertise "left_arm_gripper/goal_position" "iai_wsg_50_msgs/PositionCmd"))
  (setf (getf *neck-configuration-publisher* :right)
        (roslisp:advertise "right_arm_gripper/goal_position" "iai_wsg_50_msgs/PositionCmd")))

(defun destroy-gripper-position-publishers ()
  (setf *neck-configuration-publisher* '(:left nil :right nil)))

(roslisp-utilities:register-ros-init-function init-gripper-position-publishers)
(roslisp-utilities:register-ros-cleanup-function destroy-gripper-position-publishers)

(defun actuate-gripper (&key left-or-right position effort)
  (declare (type (or keyword list) left-or-right)
           (type number position effort))
  (roslisp:publish
   (getf *gripper-publishers* left-or-right)
   (roslisp::make-message
    'iai_wsg_50_msgs-msg:PositionCmd
    :pos position
    :speed 30.0
    :force effort)))

