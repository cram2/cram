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

(defun make-wiggle-action-client ()
  (make-simple-action-client
   'wiggle-action
   "wiggle_wiggle_wiggle" "wiggle_msgs/WiggleAction"
   60.0))

(roslisp-utilities:register-ros-init-function make-wiggle-action-client)

(defun make-wiggle-action-goal (arm goal-pose timeout)
  (declare (type cl-transforms-stamped:pose-stamped goal-pose)
           (type keyword arm)
           (type number timeout))
  (roslisp:make-message
   'wiggle_msgs-msg:WiggleGoal
   :wiggle_type (roslisp:symbol-code 'wiggle_msgs-msg:WiggleGoal :x_y)
   ;; wiggle types are :x, :y, :x_y, :angle, :x_angle, :y_angle, :x_y_angle
   :arm (roslisp:symbol-code 'wiggle_msgs-msg:WiggleGoal (ecase arm
                                                           (:left :left_arm)
                                                           (:right :right_arm)))
   :goal_pose (cl-transforms-stamped:to-msg goal-pose)
   (:data :timeout) timeout))

(defun ensure-wiggle-input (frame pose)
  (cram-tf:ensure-pose-in-frame pose frame))

(defun ensure-wiggle-output (status goal-pose goal-frame
                             convergence-delta-xy convergence-delta-theta)
  (when (eql status :timeout)
    (cpl:fail 'common-fail:actionlib-action-timed-out :description "Wiggle action timed out"))
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Wiggle action preempted.")
    (return-from ensure-wiggle-output))
  (unless (cram-tf:tf-frame-converged goal-frame goal-pose
                                      convergence-delta-xy convergence-delta-theta)
    (cpl:fail 'common-fail:manipulation-goal-not-reached
              :description (format nil "Wiggle action did not converge to goal: ~
                                        ~a should have been at ~a ~
                                        with delta-xy of ~a and delta-angle of ~a."
                                   goal-frame goal-pose
                                   convergence-delta-xy convergence-delta-theta))))

(defun move-arm-wiggle (&key
                          arm goal-pose (duration 5.0)
                          ;; TODO: get rid of 5 when Simon updates the action interface
                          action-timeout
                          (pose-base-frame cram-tf:*robot-base-frame*)
                          (left-tool-frame cram-tf:*robot-left-tool-frame*)
                          (right-tool-frame cram-tf:*robot-right-tool-frame*)
                          (convergence-delta-xy *giskard-convergence-delta-xy*)
                          (convergence-delta-theta *giskard-convergence-delta-theta*))
  (declare (type cl-transforms-stamped:pose-stamped goal-pose)
           (type (or null string) pose-base-frame left-tool-frame right-tool-frame)
           (type (or null number)
                 duration action-timeout convergence-delta-xy convergence-delta-theta)
           (type keyword arm))
  (let ((goal-pose (ensure-wiggle-input pose-base-frame goal-pose)))
    (multiple-value-bind (result status)
        (call-simple-action-client 'wiggle-action
                                   :action-goal (make-wiggle-action-goal arm goal-pose
                                                                         duration)
                                   :action-timeout action-timeout)
      (ensure-wiggle-output status goal-pose (ecase arm
                                               (:left left-tool-frame)
                                               (:right right-tool-frame))
                            convergence-delta-xy convergence-delta-theta)
      (values result status))))
