;;;
;;; Copyright (c) 2022, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :tiago-pm)

(defparameter *gripper-action-timeout* 20.0 "in seconds")

(defparameter *gripper-convergence-delta* 0.005 "in meters")
(defparameter *gripper-gripped-min-position* 0.007 "In meters.")
(defparameter *gripper-joint-limit-tiny-offset* 0.00
  "In meters, not to go too close to joint limits.")

(cpm:def-process-module grippers-pm (motion-designator)
  (destructuring-bind (command action-type-or-position which-gripper &optional effort)
      (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-gripper-joint
       (call-gripper-action
        :action-type-or-position action-type-or-position
        :arm which-gripper
        :effort effort
        :check-goal t
        :action-timeout *gripper-action-timeout*)))))


(defun make-gripper-action-clients ()
  (actionlib-client:make-simple-action-client
   'right-gripper
   "gripper_right_controller/increment"
   "teleop_tools_msgs/IncrementAction"
   *gripper-action-timeout*)

  (actionlib-client:make-simple-action-client
   'left-gripper
   "gripper_left_controller/increment"
   "teleop_tools_msgs/IncrementAction"
   *gripper-action-timeout*))

(roslisp-utilities:register-ros-init-function make-gripper-action-clients)

(defun make-gripper-action-goal (deltas)
  (roslisp:make-message
   'teleop_tools_msgs-msg:incrementgoal
   :increment_by (map 'vector #'identity deltas)))

(defun call-one-gripper-action (arm deltas action-timeout)
  (actionlib-client:call-simple-action-client
   (if (eq arm :left)
       'left-gripper
       'right-gripper)
   :action-goal (make-gripper-action-goal deltas)
   :action-timeout action-timeout))

(defun ensure-gripper-goal-input (action-type-or-position arm)
  (declare (type (or number keyword) action-type-or-position)
           (type (or keyword list) arm))
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
      (error "[tiago gripper] Robot gripper joint was not defined."))
    (let* ((position
             (etypecase action-type-or-position
               (number
                (cond
                  ((< action-type-or-position gripper-lower-limit)
                   (roslisp:ros-warn (giskard gripper)
                                     "POSITION (~a) cannot be < ~a. Clipping."
                                     action-type-or-position gripper-lower-limit)
                   (+ gripper-lower-limit *gripper-joint-limit-tiny-offset*))
                  ((> action-type-or-position gripper-upper-limit)
                   (roslisp:ros-warn (giskard gripper)
                                     "POSITION (~a) shouldn't be > ~a. Clipping."
                                     action-type-or-position gripper-upper-limit)
                   (- gripper-upper-limit *gripper-joint-limit-tiny-offset*))
                  (t
                   ;; in case the gripper is commanded in radian or so
                   (* gripper-multiplier action-type-or-position))))
               (keyword
                (ecase action-type-or-position
                  (:open (- gripper-upper-limit *gripper-joint-limit-tiny-offset*))
                  (:close (+ gripper-lower-limit *gripper-joint-limit-tiny-offset*))
                  ;; (:grip (+ gripper-lower-limit *gripper-joint-limit-tiny-offset*))
                  (:grip -1.0)))))
           (gripper-joints
             (mapcar (lambda (bindings)
                       (cut:var-value '?gripper-joint bindings))
                     (cut:force-ll
                      (prolog:prolog
                       `(and (rob-int:robot ?robot)
                             (rob-int:gripper-joint ?robot ,arm ?gripper-joint))))))
           (current-states
             (joints:joint-positions gripper-joints))
           (current-state-left
             (first current-states))
           (current-state-right
             (second current-states))
           (delta-left
             (- position current-state-left))
           (delta-right
             (- position current-state-right)))
      (list delta-left delta-right position))))

(defun ensure-gripper-goal-reached (arm action-type-or-position joint-angle)
  (when (and arm action-type-or-position joint-angle)
    (let ((gripper-joints
            (mapcar (lambda (bindings)
                      (cut:var-value '?gripper-joint bindings))
                    (cut:force-ll
                     (prolog:prolog
                      `(and (rob-int:robot ?robot)
                            (rob-int:gripper-joint ?robot ,arm ?gripper-joint)))))))
      (when (cut:is-var gripper-joints)
        (error "[tiago-pm] Robot gripper joint was not defined."))
      (let ((current-states (joints:joint-positions gripper-joints))
            (goal-state joint-angle))
        ;; (roslisp:ros-info (tiago-pm grippers) "Gripper current state: ~a, ~
        ;;                                        gripper goal: ~a."
        ;;                       current-states goal-state)
        (if (and (symbolp action-type-or-position)
                 (eq action-type-or-position :grip))
            (when (< (/ (+ (first current-states) (second current-states)) 2.0)
                     *gripper-gripped-min-position*)
              (cpl:fail
               (make-instance 'common-fail:gripper-closed-completely
                 :description (format nil "Gripper ~a should have gripped,~%~
                                          but it closed completely to ~a,~%~
                                          which is below ~a."
                                      arm current-states
                                      *gripper-gripped-min-position*))))
            (unless (cram-tf:values-converged current-states
                                              (list goal-state goal-state)
                                              *gripper-convergence-delta*)
              (cpl:fail
               (make-instance 'common-fail:gripper-goal-not-reached
                 :description (format nil "Gripper did not converge to goal:~%~
                                      arm: ~a, current state: ~a, goal state: ~a, ~
                                      delta joint: ~a."
                                      arm current-states goal-state
                                      *gripper-convergence-delta*)))))
        (roslisp:ros-info (tiago-pm grippers) "Gripper goal reached.")))))


(defun call-gripper-action (&key
                              action-timeout
                              action-type-or-position arm effort
                              check-goal)
  (declare (type (or number null) action-timeout)
           (type (or keyword list) arm)
           (type (or number keyword) action-type-or-position)
           (ignore effort))

  (unless (listp arm)
    (setf arm (list arm)))

  (cpl:with-retry-counters ((gripping-retries 1))
    (cpl:with-failure-handling

        ((common-fail:gripper-low-level-failure (e)
           (roslisp:ros-warn (tiago-pm grippers) "~a~%" e)
           (cpl:do-retry gripping-retries
             (roslisp:ros-warn (tiago-pm grippers) "Retrying.")
             (cpl:sleep 1.0)
             (cpl:retry))
           (roslisp:ros-warn (tiago-pm grippers) "Failing to action level.")))

      (mapcar (lambda (arm-element)
                (let ((offset-left-and-right-and-position
                        (ensure-gripper-goal-input action-type-or-position arm-element)))
                  (multiple-value-bind (result status)

                      (call-one-gripper-action
                       arm-element
                       (subseq offset-left-and-right-and-position 0 2)
                       action-timeout)

                    (when (and result status check-goal)
                      (cpl:sleep 1.5)

                      (ensure-gripper-goal-reached
                       arm-element
                       action-type-or-position
                       (third offset-left-and-right-and-position))
                      :goal-not-achieved-yet))))
              arm))))
