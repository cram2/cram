;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(defparameter *max-joint-velocity* 0.5 "in radians/sec")

(defparameter *joint-action-timeout* 5.0
  "How many seconds to wait before returning from joint action.")

(defparameter *joint-max-velocity* (cram-math:degrees->radians 30))
(defparameter *joint-impedance-list* '(500 500 500 500 500 1500 1500))

(actionlib-client:make-simple-action-client
   'dlr-joint-action
   "left_arm_joint"
   "iai_dlr_msgs/JointAction"
   *joint-action-timeout*)

;; (defun make-joint-action-client ())
;; (roslisp-utilities:register-ros-init-function make-joint-action-client)

(defun make-joint-trajectory-action-goal (joint-states-list)
  (roslisp:make-message
   'iai_dlr_msgs-msg:JointGoal
   :points (map 'vector
                (lambda (joint-states)
                  (roslisp:make-msg
                   'iai_dlr_msgs-msg:positiontrajectorypoint
                   :positions (apply #'vector
                                     (mapcar #'second joint-states))))
                joint-states-list)
   :max_vel *joint-max-velocity*
   :joint_imp (apply #'vector *joint-impedance-list*)))

(defun move-arm-joints (&key
                          goal-joint-states-left
                          goal-joint-states-right
                          (action-timeout *joint-action-timeout*))
  (declare (ignore goal-joint-states-right))
  (unless (listp (caar goal-joint-states-left))
    (setf goal-joint-states-left (list goal-joint-states-left)))
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'dlr-joint-action
       :action-goal (make-joint-trajectory-action-goal goal-joint-states-left)
       :action-timeout action-timeout)
    (roslisp:ros-info (joint-trajectory-action) "left arm action finished.")
    (values result status)))

(defun move-arm-joints-example ()
  (move-arm-joints
   :goal-joint-states-left
   '((("1" 0) ("2" 0.35) ("3" 0) ("4" -1.85) ("5" 0) ("6" 0) ("7" 0))
     (("1" 0) ("2" 0.35) ("3" 0) ("4" -1.85) ("5" 0) ("6" 0.1) ("7" 0.1))
     (("1" 0) ("2" 0.35) ("3" 0) ("4" -1.85) ("5" 0) ("6" 0.2) ("7" 0.2))
     (("1" 0) ("2" 0.35) ("3" 0) ("4" -1.85) ("5" 0) ("6" -0.1) ("7" -0.1))
     (("1" 0) ("2" 0.35) ("3" 0) ("4" -1.85) ("5" 0) ("6" -0.1) ("7" -0.3))
     (("1" 0) ("2" 0.35) ("3" 0) ("4" -1.85) ("5" 0) ("6" 0.2) ("7" 0.4)))))
