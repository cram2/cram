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

(in-package :boxy-ll)

(defparameter *giskard-convergence-delta-joint* 0.17 "in radiants, about 10 degrees")

;; (defun make-giskard-joint-convergence (arm convergence-delta-joint)
;;   (let ((prefix (ecase arm
;;                   (:right "right")
;;                   (:left "left"))))
;;     (map 'vector (lambda (error-name)
;;                    (roslisp:make-message
;;                     "giskard_msgs/semanticfloat64"
;;                     :semantics (concatenate 'string prefix error-name)
;;                     :value convergence-delta-joint))
;;          '("_arm_0_error"
;;            "_arm_1_error"
;;            "_arm_2_error"
;;            "_arm_3_error"
;;            "_arm_4_error"
;;            "_arm_5_error"
;;            "_arm_6_error"))))

(defun make-giskard-joint-action-goal (left-configuration right-configuration
                                       &optional convergence-delta-joint)
  (declare (type list left-configuration right-configuration)
           (ignore convergence-delta-joint))
  (roslisp:make-message
   'giskard_msgs-msg:WholeBodyGoal
   (:type :command)
   (roslisp:symbol-code 'giskard_msgs-msg:wholebodycommand :standard_controller)
   (:type :left_ee :command)
   (roslisp:symbol-code 'giskard_msgs-msg:armcommand :joint_goal)
   (:goal_configuration :left_ee :command)
   (apply #'vector left-configuration)
   ;; (:convergence_thresholds :left_ee :command)
   ;; (if convergence-delta-joint
   ;;     (make-giskard-joint-convergence :left convergence-delta-joint)
   ;;     (vector))
   (:type :right_ee :command)
   (roslisp:symbol-code 'giskard_msgs-msg:armcommand :joint_goal)
   (:goal_configuration :right_ee :command)
   (apply #'vector right-configuration)
   ;; (:convergence_thresholds :right_ee :command)
   ;; (if convergence-delta-joint
   ;;     (make-giskard-joint-convergence :right convergence-delta-joint)
   ;;     (vector))
   ))

(defun get-arm-joint-states (arm)
  (joint-positions
   (cut:var-value '?joints
                  (cut:lazy-car
                   (prolog:prolog
                    `(cram-robot-interfaces:arm-joints boxy-descr:boxy ,arm ?joints))))
   *robot-joint-states-msg*))

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
  (when (eql status :timeout)
    (cpl:fail 'common-fail:actionlib-action-timed-out :description "Giskard action timed out"))
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Giskard action preempted.")
    (return-from ensure-giskard-joint-goal-reached))
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

(defun move-arms-giskard-joint (&key
                                  goal-configuration-left goal-configuration-right action-timeout
                                  (convergence-delta-joint *giskard-convergence-delta-joint*))
  (declare (type list goal-configuration-left goal-configuration-right)
           (type (or null number) action-timeout convergence-delta-joint))
  (multiple-value-bind (goal-configuration-left goal-configuration-right)
      (ensure-giskard-joint-input-parameters goal-configuration-left goal-configuration-right)
    (multiple-value-bind (result status)
        (call-simple-action-client
         'giskard-action
         :action-goal (make-giskard-joint-action-goal
                       goal-configuration-left goal-configuration-right
                       ;; convergence-delta-joint
                       )
         :action-timeout action-timeout)
      (ensure-giskard-joint-goal-reached status goal-configuration-left goal-configuration-right
                                         convergence-delta-joint)
      (values result status))))
