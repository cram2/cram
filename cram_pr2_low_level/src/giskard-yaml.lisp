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

(defun make-giskard-yaml-convergence (key-value-pairs)
  (apply #'vector (mapcar (lambda (key-value-pair)
                            (destructuring-bind (name-string value)
                                key-value-pair
                              (roslisp:make-message
                               "giskard_msgs/semanticfloat64"
                               :semantics name-string
                               :value value)))
                          key-value-pairs)))

(defun make-giskard-yaml-action-goal (yaml-string convergence-key-value-pairs)
  (declare (ignore convergence-key-value-pairs)
           (type (or null string) yaml-string))
  (actionlib:make-action-goal
      (get-giskard-action-client)
    (:type :command)
    (roslisp:symbol-code 'giskard_msgs-msg:wholebodycommand :yaml_controller)
    (:yaml_spec :command) yaml-string
    ;; (:convergence_thresholds :command)
    ;; (make-giskard-yaml-convergence convergence-key-value-pairs)
    ))

(defun ensure-giskard-yaml-input-parameters (yaml-string convergence-key-value-pairs)
  (values yaml-string convergence-key-value-pairs))
;;   (values (cram-tf:ensure-pose-in-frame (or left-goal
;;                                     (cl-transforms-stamped:transform-pose-stamped
;;                                      cram-tf:*transformer*
;;                                      :timeout cram-tf:*tf-default-timeout*
;;                                      :target-frame frame
;;                                      :pose (cl-transforms-stamped:pose->pose-stamped
;;                                             *left-tool-frame*
;;                                             0.0
;;                                             (cl-transforms:make-identity-pose))))
;;                                 frame)
;;           (cram-tf:ensure-pose-in-frame (or right-goal
;;                                     (cl-transforms-stamped:transform-pose-stamped
;;                                      cram-tf:*transformer*
;;                                      :timeout cram-tf:*tf-default-timeout*
;;                                      :target-frame frame
;;                                      :pose (cl-transforms-stamped:pose->pose-stamped
;;                                             *right-tool-frame*
;;                                             0.0
;;                                             (cl-transforms:make-identity-pose))))
;;                                 frame)))

(defun ensure-giskard-yaml-goal-reached (status)
  (when (eql status :timeout)
    (cpl:fail 'common-fail:actionlib-action-timed-out
              :description "Giskard action timed out"))
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Giskard action preempted.")
    (return-from ensure-giskard-yaml-goal-reached))
  ;; (when goal-position-left
;;     (unless (cram-tf:tf-frame-converged goal-frame-left goal-position-left
;;                                 convergence-delta-xy convergence-delta-theta)
;;       (cpl:fail 'pr2-low-level-failure
;;                 :description (format nil "Giskard did not converge to goal:
;; ~a should have been at ~a with delta-xy of ~a and delta-angle of ~a."
;;                                      goal-frame-left goal-position-left
;;                                      convergence-delta-xy convergence-delta-theta))))
;;   (when goal-position-right
;;     (unless (cram-tf:tf-frame-converged goal-frame-right goal-position-right
;;                                 convergence-delta-xy convergence-delta-theta)
;;       (cpl:fail 'pr2-low-level-failure
;;                 :description (format nil "Giskard did not converge to goal:
;; ~a should have been at ~a with delta-xy of ~a and delta-angle of ~a."
;;                                      goal-frame-right goal-position-right
;;                                      convergence-delta-xy convergence-delta-theta))))
  )

(defun call-giskard-yaml-action (&key yaml-string convergence-key-value-pairs
                                   (action-timeout *giskard-action-timeout*))
  (declare (type (or null string) yaml-string)
           (type list convergence-key-value-pairs))
  (multiple-value-bind (yaml-string convergence-key-value-pairs)
      (ensure-giskard-yaml-input-parameters yaml-string convergence-key-value-pairs)
    (multiple-value-bind (result status)
        (cpl:with-failure-handling
            ((simple-error (e)
               (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
               (init-giskard-action-client)
               (cpl:retry)))
          (let ((actionlib:*action-server-timeout* 10.0))
            (actionlib:call-goal
             (get-giskard-action-client)
             (make-giskard-yaml-action-goal
              yaml-string convergence-key-value-pairs)
             :timeout action-timeout)))
      (ensure-giskard-yaml-goal-reached status)
      (values result status))))
