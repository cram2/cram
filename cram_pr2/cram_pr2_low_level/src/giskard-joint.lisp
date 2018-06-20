;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-ll)

(defun make-giskard-joint-convergence (arm convergence-delta-joint)
  (let ((prefix (ecase arm
                  (:right "r")
                  (:left "l"))))
    (map 'vector (lambda (error-name)
                   (roslisp:make-message
                    "giskard_msgs/semanticfloat64"
                    :semantics (concatenate 'string prefix error-name)
                    :value convergence-delta-joint))
         '("_shoulder_pan_error"
           "_shoulder_lift_error"
           "_upper_arm_roll_error"
           "_elbow_flex_error"
           "_forearm_roll_error"
           "_wrist_flex_error"
           "_wrist_roll_error"))))

(defun make-giskard-joint-action-goal (left-configuration right-configuration
                                       &optional convergence-delta-joint)
  (declare (type list left-configuration right-configuration))
  (actionlib:make-action-goal
      (get-giskard-action-client)
    (:type :command)
    (roslisp:symbol-code 'giskard_msgs-msg:wholebodycommand :standard_controller)
    (:type :left_ee :command)
    (roslisp:symbol-code 'giskard_msgs-msg:armcommand :joint_goal)
    (:goal_configuration :left_ee :command)
    (apply #'vector left-configuration)
    (:convergence_thresholds :left_ee :command)
    (if convergence-delta-joint
        (make-giskard-joint-convergence :left convergence-delta-joint)
        (vector))
    (:type :right_ee :command)
    (roslisp:symbol-code 'giskard_msgs-msg:armcommand :joint_goal)
    (:goal_configuration :right_ee :command)
    (apply #'vector right-configuration)
    (:convergence_thresholds :right_ee :command)
    (if convergence-delta-joint
        (make-giskard-joint-convergence :right convergence-delta-joint)
        (vector))))

(defun ensure-giskard-joint-input-parameters (left-goal right-goal)
  (flet ((ensure-giskard-joint-goal (goal arm)
           (if (and (listp goal) (= (length goal) 7))
               goal
               (and (roslisp:ros-warn (low-level giskard)
                                      "Joint goal ~a was not a list of 7. Ignoring."
                                      goal)
                    (get-arm-joint-states arm)))))
   (values (ensure-giskard-joint-goal left-goal :left)
           (ensure-giskard-joint-goal right-goal :right))))

(defun ensure-giskard-joint-goal-reached (status
                                          goal-configuration-left goal-configuration-right
                                          convergence-delta-joint)
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Giskard action preempted.")
    (return-from ensure-giskard-joint-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (pr2-ll giskard-joint) "Giskard action timed out."))
  (flet ((ensure-giskard-joint-arm-goal-reached (arm goal-configuration)
           (let ((configuration (get-arm-joint-states arm)))
             (unless (values-converged (normalize-joint-angles configuration)
                                       (normalize-joint-angles goal-configuration)
                                       convergence-delta-joint)
               (cpl:fail 'common-fail:manipulation-goal-not-reached
                         :description (format nil "Giskard did not converge to goal:
~a (~a) should have been at ~a with delta-joint of ~a."
                                              arm
                                              (normalize-joint-angles configuration)
                                              (normalize-joint-angles goal-configuration)
                                              convergence-delta-joint))))))
    (when goal-configuration-left
      (ensure-giskard-joint-arm-goal-reached :left goal-configuration-left))
    (when goal-configuration-right
      (ensure-giskard-joint-arm-goal-reached :right goal-configuration-right))))

(defun call-giskard-joint-action (&key
                                    left right
                                    (action-timeout *giskard-action-timeout*)
                                    (convergence-delta-joint *giskard-convergence-delta-joint*))
  (declare (type list left right)
           (type (or null number) action-timeout convergence-delta-joint))
  (multiple-value-bind (left-goal right-goal)
      (ensure-giskard-joint-input-parameters left right)
    (multiple-value-bind (result status)
        (cpl:with-failure-handling
            ((simple-error (e)
               (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
               (init-giskard-action-client)
               (cpl:retry)))
          (let ((actionlib:*action-server-timeout* 10.0))
            (actionlib:call-goal
             (get-giskard-action-client)
             (make-giskard-joint-action-goal left-goal right-goal
;                                             convergence-delta-joint
                                             )
             :timeout action-timeout)))
      (ensure-giskard-joint-goal-reached status left-goal right-goal
                                         convergence-delta-joint)
      (values result status))))
