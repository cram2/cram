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

(defparameter *neck-converngence-delta-joint-vel* 0.00001 "in radiants/sec")

(defvar *neck-configuration-publisher* nil "ROS publisher for MoveIT desired_joints message.")

(defun init-neck-configuration-publisher ()
  (setf *neck-configuration-publisher*
        (roslisp:advertise "desired_joints" "iai_control_msgs/pose_w_joints")))

(defun destroy-neck-configuration-publisher ()
  (setf *neck-configuration-publisher* nil))

(roslisp-utilities:register-ros-init-function init-neck-configuration-publisher)
(roslisp-utilities:register-ros-cleanup-function destroy-neck-configuration-publisher)

(defun move-neck-joint (&key goal-configuration)
  (declare (type list goal-configuration))
  "Neck has 6 joints, so as `goal-configuration' use a list of length 6."
  (format t "MOVING NECK~%")
  (roslisp::publish *neck-configuration-publisher*
                    (roslisp::make-message
                     'iai_control_msgs-msg:pose_w_joints
                     :joint_values (map 'vector #'identity goal-configuration)))
  (cpl:sleep 3.0)
  (flet ((goal-reached (state-msg)
           (values-converged
            (joint-velocities '("neck_shoulder_pan_joint" "neck_shoulder_lift_joint"
                                "neck_elbow_joint" "neck_wrist_1_joint" "neck_wrist_2_joint"
                                "neck_wrist_3_joint")
                              state-msg)
            '(0.0 0.0 0.0 0.0 0.0 0.0)
            *neck-converngence-delta-joint-vel*)))
    (let ((reached-fluent (cpl:fl-funcall #'goal-reached *robot-joint-states-msg*)))
      (cpl:wait-for reached-fluent)
      (print "NECK REACHED"))))
