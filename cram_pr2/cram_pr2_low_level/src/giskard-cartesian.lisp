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

(defun make-giskard-cartesian-convergence (arm
                                           convergence-delta-xy convergence-delta-theta)
  (let ((prefix (ecase arm
                  (:right "r")
                  (:left "l"))))
    (vector
     (roslisp:make-message
      "giskard_msgs/semanticfloat64"
      :semantics (concatenate 'string prefix "_trans_error")
      :value convergence-delta-xy)
     (roslisp:make-message
      "giskard_msgs/semanticfloat64"
      :semantics (concatenate 'string prefix "_rot_error")
      :value convergence-delta-theta))))

(defun make-giskard-cartesian-action-goal (left-pose right-pose
                                           convergence-delta-xy convergence-delta-theta)
  (declare (ignore convergence-delta-xy convergence-delta-theta)
           (type (or cl-transforms:pose cl-transforms-stamped:pose-stamped)
                 left-pose right-pose))
  (actionlib:make-action-goal
      (get-giskard-action-client)
    (:type :command)
    (roslisp:symbol-code 'giskard_msgs-msg:wholebodycommand :standard_controller)
    (:type :left_ee :command)
    (roslisp:symbol-code 'giskard_msgs-msg:armcommand :cartesian_goal)
    (:goal_pose :left_ee :command)
    (cl-transforms-stamped:to-msg left-pose)
    ;; (:convergence_thresholds :left_ee :command)
    ;; (make-giskard-cartesian-convergence :left convergence-delta-xy convergence-delta-theta)
    (:type :right_ee :command)
    (roslisp:symbol-code 'giskard_msgs-msg:armcommand :cartesian_goal)
    (:goal_pose :right_ee :command)
    (cl-transforms-stamped:to-msg right-pose)
    ;; (:convergence_thresholds :right_ee :command)
    ;; (make-giskard-cartesian-convergence :right convergence-delta-xy convergence-delta-theta)
    ))

(defun ensure-giskard-cartesian-input-parameters (frame left-goal right-goal)
  (values (cram-tf:ensure-pose-in-frame (or left-goal
                                    (cl-transforms-stamped:transform-pose-stamped
                                     cram-tf:*transformer*
                                     :timeout cram-tf:*tf-default-timeout*
                                     :target-frame frame
                                     :pose (cl-transforms-stamped:pose->pose-stamped
                                            *left-tool-frame*
                                            0.0
                                            (cl-transforms:make-identity-pose))
                                     :use-current-ros-time t))
                                frame)
          (cram-tf:ensure-pose-in-frame (or right-goal
                                    (cl-transforms-stamped:transform-pose-stamped
                                     cram-tf:*transformer*
                                     :timeout cram-tf:*tf-default-timeout*
                                     :target-frame frame
                                     :pose (cl-transforms-stamped:pose->pose-stamped
                                            *right-tool-frame*
                                            0.0
                                            (cl-transforms:make-identity-pose))
                                     :use-current-ros-time t))
                                frame)))

(defun ensure-giskard-cartesian-goal-reached (status goal-position-left goal-position-right
                                              goal-frame-left goal-frame-right
                                              convergence-delta-xy convergence-delta-theta)
  (when (eql status :timeout)
    (cpl:fail 'common-fail:actionlib-action-timed-out :description "Giskard action timed out"))
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Giskard action preempted.")
    (return-from ensure-giskard-cartesian-goal-reached))
  (when goal-position-left
    (unless (cram-tf:tf-frame-converged goal-frame-left goal-position-left
                                        convergence-delta-xy convergence-delta-theta)
      (cpl:fail 'common-fail:manipulation-goal-not-reached
                :description (format nil "Giskard did not converge to goal:
~a should have been at ~a with delta-xy of ~a and delta-angle of ~a."
                                     goal-frame-left goal-position-left
                                     convergence-delta-xy convergence-delta-theta))))
  (when goal-position-right
    (unless (cram-tf:tf-frame-converged goal-frame-right goal-position-right
                                convergence-delta-xy convergence-delta-theta)
      (cpl:fail 'common-fail:manipulation-goal-not-reached
                :description (format nil "Giskard did not converge to goal:
~a should have been at ~a with delta-xy of ~a and delta-angle of ~a."
                                     goal-frame-right goal-position-right
                                     convergence-delta-xy convergence-delta-theta)))))

(defun call-giskard-cartesian-action (&key
                                        left right
                                        (ik-base-frame *robot-base-frame*)
                                        (action-timeout *giskard-action-timeout*)
                                        (left-tool-frame *left-tool-frame*)
                                        (right-tool-frame *right-tool-frame*)
                                        (convergence-delta-xy *giskard-convergence-delta-xy*)
                                        (convergence-delta-theta *giskard-convergence-delta-theta*))
  (declare (type (or null cl-transforms:pose cl-transforms-stamped:pose-stamped) left right)
           (type (or null string) ik-base-frame)
           (type (or null number) action-timeout convergence-delta-xy convergence-delta-theta))
  (multiple-value-bind (left-goal right-goal)
      (ensure-giskard-cartesian-input-parameters ik-base-frame left right)
    (multiple-value-bind (result status)
        (cpl:with-failure-handling
            ((simple-error (e)
               (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
               (init-giskard-action-client)
               (cpl:retry)))
          (let ((actionlib:*action-server-timeout* 10.0))
            (actionlib:call-goal
             (get-giskard-action-client)
             (make-giskard-cartesian-action-goal
              left-goal right-goal
              convergence-delta-xy convergence-delta-theta)
             :timeout action-timeout)))
      (ensure-giskard-cartesian-goal-reached status left-goal right-goal
                                             left-tool-frame right-tool-frame
                                             convergence-delta-xy convergence-delta-theta)
      (values result status))))

