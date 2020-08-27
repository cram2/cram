;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2020, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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

(defparameter *gripper-convergence-delta-joint* 0.005 "In meters.")

(defun make-gripper-action-goal (arm joint-angle effort)
  (declare (type keyword arm)
           (type number joint-angle)
           (ignore effort))
  (make-giskard-goal
   :joint-constraints (make-gripper-joint-state-constraint arm joint-angle)
   :collisions (make-allow-all-collision)))

(defun ensure-gripper-goal-input (action-type-or-position arm effort)
  (declare (type (or number keyword) action-type-or-position)
           (type keyword arm)
           (type (or number null) effort))
  (let* ((bindings
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:gripper-joint ?robot ,arm ?gripper-joint)
                       (rob-int:joint-lower-limit ?robot ?gripper-joint ?lower-limit)
                       (rob-int:joint-upper-limit ?robot ?gripper-joint ?upper-limit)
                       (rob-int:gripper-meter-to-joint-multiplier ?robot ?mult)))))
         (gripper-joint
           (cut:var-value '?gripper-joint bindings))
         (gripper-lower-limit
           (cut:var-value '?lower-limit bindings))
         (gripper-upper-limit
           (cut:var-value '?upper-limit bindings))
         (gripper-multiplier
           (cut:var-value '?mult bindings)))
    (when (cut:is-var gripper-joint)
      (error "[giskard] Robot gripper joint was not defined."))
    (let ((position
            (etypecase action-type-or-position
              (number
               (cond
                 ((< action-type-or-position gripper-lower-limit)
                  (roslisp:ros-warn (giskard gripper)
                                    "POSITION (~a) cannot be < ~a. Clipping."
                                    action-type-or-position gripper-lower-limit)
                  gripper-lower-limit)
                 ((> action-type-or-position gripper-upper-limit)
                  (roslisp:ros-warn (giskard gripper)
                                    "POSITION (~a) shouldn't be > ~a. Clipping."
                                    action-type-or-position gripper-upper-limit)
                  gripper-upper-limit)
                 (t
                  ;; in case the gripper is commanded in radian or so
                  (* gripper-multiplier action-type-or-position))))
              (keyword
               (ecase action-type-or-position
                 (:open gripper-upper-limit)
                 (:close gripper-lower-limit)
                 (:grip gripper-lower-limit)))))
          (effort
            (or effort
                (etypecase action-type-or-position
                  (number 30.0)
                  (keyword (ecase action-type-or-position
                             (:open 30.0)
                             (:close 30.0)
                             (:grip 15.0)))))))
      (list position effort))))

(defun ensure-gripper-goal-reached (arm joint-angle)
  (when (and joint-angle arm)
    (let ((gripper-joint
            (cut:var-value
             '?gripper-joint
             (car (prolog:prolog
                   `(and (rob-int:robot ?robot)
                         (rob-int:gripper-joint ?robot ,arm ?gripper-joint)))))))
      (when (cut:is-var gripper-joint)
        (error "[giskard] Robot gripper joint was not defined."))
      (let ((current-state (car (joints:joint-positions (list gripper-joint))))
            (goal-state joint-angle))
        (unless (cram-tf:values-converged current-state
                                          goal-state
                                          *gripper-convergence-delta-joint*)
          (make-instance 'common-fail:manipulation-goal-not-reached
            :description (format nil "Giskard did not converge to goal:~%~
                                      arm: ~a, current state: ~a, goal state: ~a, ~
                                      delta joint: ~a."
                                 arm current-state goal-state
                                 *gripper-convergence-delta-joint*)))))))

(defun call-gripper-action (&key
                              action-timeout
                              action-type-or-position arm effort)
  (declare (type (or number null) action-timeout effort)
           (type keyword arm)
           (type (or number keyword) action-type-or-position))

  (let* ((position-and-effort
           (ensure-gripper-goal-input action-type-or-position arm effort))
         (goal-joint-angle
           (first position-and-effort))
         (effort
           (second position-and-effort)))

    (call-action
     :action-goal (make-gripper-action-goal arm goal-joint-angle effort)
     :action-timeout action-timeout
     ;; :check-goal-function (lambda ()
     ;;                        (ensure-gripper-goal-reached arm goal-joint-angle))
     )))
