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

(in-package :giskard)

(defparameter *torso-convergence-delta-joint* 0.01 "in meters")

(defun make-torso-action-goal (joint-angle)
  (declare (type number joint-angle))
  (make-giskard-goal
   :joint-constraints (make-joint-constraint
                       `((,cram-tf:*robot-torso-joint*) (,joint-angle)))))

(defun ensure-torso-goal-input (position)
  (declare (type (or keyword number) position))
  (let* ((bindings
           (car
            (prolog:prolog
             `(and (rob-int:robot ?robot)
                   (rob-int:robot-torso-link-joint ?robot ?_ ?joint)
                   (rob-int:joint-lower-limit ?robot ?joint ?lower)
                   (rob-int:joint-upper-limit ?robot ?joint ?upper)))))
         (lower-limit
           (cut:var-value '?lower bindings))
         (upper-limit
           (cut:var-value '?upper bindings))
         (torso-joint
           (cut:var-value '?joint bindings)))
    (if (and (not (cut:is-var lower-limit))
             (not (cut:is-var upper-limit))
             lower-limit
             upper-limit)
        (progn
          ;; don't push the robot so hard
          (setf lower-limit (+ lower-limit 0.01)
                upper-limit (- upper-limit 0.01))
          (etypecase position
            (number (if (< position lower-limit)
                        lower-limit
                        (if (> position upper-limit)
                            upper-limit
                            position)))
            (keyword (ecase position
                       (:upper-limit upper-limit)
                       (:lower-limit lower-limit)
                       (:middle (/ (- upper-limit lower-limit) 2))))))
        (or (car (joints:joint-positions (list torso-joint))) 0.0))))

(defun ensure-torso-goal-reached (goal)
  (let ((current-position
          (car (joints:joint-positions (list cram-tf:*robot-torso-joint*)))))
    (when current-position
      (unless (cram-tf:values-converged
               current-position goal *torso-convergence-delta-joint*)
        (make-instance 'common-fail:torso-goal-not-reached
          :description (format nil "Giskard did not converge to torso goal:~
                                    goal: ~a, current: ~a, delta: ~a."
                               goal current-position
                               *torso-convergence-delta-joint*))))))

(defun call-torso-action (&key action-timeout goal-joint-state)
  (declare (type (or number keyword) goal-joint-state)
           (type (or number null) action-timeout))

  (setf goal-joint-state (ensure-torso-goal-input goal-joint-state))

  (call-action
   :action-goal (make-torso-action-goal goal-joint-state)
   :action-timeout action-timeout
   :check-goal-function (lambda (result status)
                          (declare (ignore result status))
                          (ensure-torso-goal-reached goal-joint-state))))
