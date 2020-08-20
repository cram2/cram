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

(defparameter *xy-goal-tolerance* 0.05 "in meters")
(defparameter *yaw-goal-tolerance* 0.1 "in radiants, about 6 degrees.")

(defun make-giskard-base-action-goal (pose)
  (declare (type cl-transforms-stamped:pose-stamped pose))
  (make-giskard-goal
   :cartesian-constraints (make-simple-cartesian-constraint
                           cram-tf:*odom-frame* cram-tf:*robot-base-frame* pose)
   :joint-constraints (make-current-joint-state-constraint '(:left :right))
   :collisions (make-avoid-all-collision)))

(defun ensure-giskard-base-input-parameters (pose)
  (cram-tf:ensure-pose-in-frame pose cram-tf:*fixed-frame*))

(defun ensure-giskard-base-goal-reached (result status goal
                                         convergence-delta-xy convergence-delta-theta)
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard)
                      "Giskard action preempted with result ~a" result)
    (return-from ensure-giskard-base-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (pr2-ll giskard-cart) "Giskard action timed out."))
  (when (eql status :aborted)
    (roslisp:ros-warn (pr2-ll giskard-cart)
                      "Giskard action aborted! With result ~a" result)
    ;; (cpl:fail 'common-fail:manipulation-goal-not-reached
    ;;           :description "Giskard did not converge to goal because of collision")
    )
  (unless (cram-tf:tf-frame-converged cram-tf:*robot-base-frame* goal
                                      convergence-delta-xy convergence-delta-theta)
    (cpl:fail 'common-fail:navigation-goal-not-reached
              :description (format nil "Giskard did not converge to goal:~%~
                                        ~a should have been at ~a ~
                                        with delta-xy of ~a and delta-angle of ~a."
                                   cram-tf:*robot-base-frame* goal
                                   convergence-delta-xy convergence-delta-theta))))

(defun call-giskard-base-action (&key
                                   goal-pose action-timeout
                                   (convergence-delta-xy *xy-goal-tolerance*)
                                   (convergence-delta-theta *yaw-goal-tolerance*))
  (declare (type cl-transforms-stamped:pose-stamped goal-pose)
           (type (or null number) action-timeout convergence-delta-xy convergence-delta-theta))
  (let ((goal-pose
          (ensure-giskard-base-input-parameters goal-pose)))
    (cram-tf:visualize-marker goal-pose :r-g-b-list '(0 1 0))
    (multiple-value-bind (result status)
        (let ((goal (make-giskard-base-action-goal goal-pose)))
          (actionlib-client:call-simple-action-client
           'giskard-action
           :action-goal goal
           :action-timeout action-timeout))
      (ensure-giskard-base-goal-reached result status goal-pose
                                        convergence-delta-xy convergence-delta-theta)
      (joints:full-joint-states-as-hash-table))))
