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

(defparameter *neck-action-timeout* 4.0
  "How many seconds to wait before returning from neck action.")

(defparameter *neck-trajectory-duration* 3.0 "in seconds")

(actionlib-client:make-simple-action-client
 'neck-action
 "neck/follow_joint_trajectory"
 "control_msgs/FollowJointTrajectoryAction"
 5.0)

(defun make-neck-action-goal (joint-states)
  (print "HELLOOOOO")
  (let ((joint-num (length joint-states)))
   (roslisp:make-message
    'control_msgs-msg:FollowJointTrajectoryGoal
    :trajectory (roslisp:make-msg
                 'trajectory_msgs-msg:JointTrajectory
                 :joint_names (apply #'vector
                                     (mapcar #'first joint-states))
                 :points (vector
                          (roslisp:make-msg
                           'trajectory_msgs-msg:JointTrajectoryPoint
                           :positions (apply #'vector (mapcar #'second joint-states))
                           :velocities (make-array joint-num :initial-element 0.0)
                           :accelerations (make-array joint-num :initial-element 0.0)
                           :effort (make-array joint-num :initial-element 0.0)
                           :time_from_start *neck-trajectory-duration*))))))

(defun move-neck-joints (&key goal-configuration
                          (action-timeout *neck-action-timeout*))
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'neck-action
       :action-goal (make-neck-action-goal goal-configuration)
       :action-timeout action-timeout)
    (roslisp:ros-info (boxy-ll neck-action) "neck action finished.")
    (values result status)))



;; (defparameter *neck-converngence-delta-joint-vel* 0.00001 "in radiants/sec")
;; (defvar *neck-configuration-publisher* nil "ROS publisher for MoveIT desired_joints message.")
;; (defun init-neck-configuration-publisher ()
;;   (setf *neck-configuration-publisher*
;;         (roslisp:advertise "desired_joints" "iai_control_msgs/pose_w_joints")))
;; (defun destroy-neck-configuration-publisher ()
;;   (setf *neck-configuration-publisher* nil))
;; (roslisp-utilities:register-ros-init-function init-neck-configuration-publisher)
;; (roslisp-utilities:register-ros-cleanup-function destroy-neck-configuration-publisher)
;; (defun move-neck-joint (&key goal-configuration)
;;   (declare (type list goal-configuration))
;;   "Neck has 6 joints, so as `goal-configuration' use a list of length 6,
;; where each element is ('joint_name' joint_value)."
;;   (format t "MOVING NECK~%")
;;   (roslisp::publish *neck-configuration-publisher*
;;                     (roslisp::make-message
;;                      'iai_control_msgs-msg:pose_w_joints
;;                      :joint_values (map 'vector #'identity
;;                                         (mapcar #'second goal-configuration))))
;;   (cpl:sleep 3.0)
;;   (format t "moved~%")
;;   (flet ((goal-reached (state-msg)
;;            (cram-tf:values-converged
;;             (joints:joint-velocities (mapcar #'first goal-configuration)
;;                               state-msg)
;;             '(0.0 0.0 0.0 0.0 0.0 0.0)
;;             *neck-converngence-delta-joint-vel*)))
;;     (let ((reached-fluent (cpl:fl-funcall #'goal-reached *robot-joint-states-msg*)))
;;       (cpl:wait-for reached-fluent)
;;       (print "NECK REACHED"))))
