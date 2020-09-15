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

(defparameter *base-convergence-delta-xy*
  0.05 "in meters.")
(defparameter *base-convergence-delta-theta*
  0.1 "in radiants, about 6 degrees.")
(defparameter *base-collision-avoidance-distance*
  0.2 "In meters.")
(defparameter *base-collision-avoidance-hint-vector*
  (cl-transforms:make-3d-vector 0 -1 0))
(defparameter *base-collision-avoidance-hint-link*
  "kitchen_island" "A link name from the environment URDF.")
(defparameter *base-max-velocity-fast-xy*
  0.5 "In meters")
(defparameter *base-max-velocity-fast-theta*
  0.2 "In rad, about 11.5 deg.")
(defparameter *base-max-velocity-slow-xy*
  0.1 "In meters")
(defparameter *base-max-velocity-slow-theta*
  0.2 "In rad, about 11.5 deg.")

(defun make-giskard-base-action-goal (pose)
  (declare (type cl-transforms-stamped:pose-stamped pose))
  (make-giskard-goal
   :constraints (list
                 (make-cartesian-constraint
                  cram-tf:*odom-frame* cram-tf:*robot-base-frame* pose
                  :avoid-collisions-much t
                  :max-velocity *base-max-velocity-fast-xy*)
                 (make-collision-avoidance-hint-constraint
                  cram-tf:*robot-base-frame*
                  *base-collision-avoidance-hint-link*
                  (cl-transforms-stamped:make-vector-stamped
                   cram-tf:*fixed-frame* 0.0
                   *base-collision-avoidance-hint-vector*))
                 (make-base-velocity-constraint
                  *base-max-velocity-fast-xy* *base-max-velocity-fast-theta*))
   :joint-constraints (make-current-joint-state-constraint '(:left :right))
   :collisions (make-avoid-all-collision *base-collision-avoidance-distance*)))

(defun ensure-base-goal-input (pose)
  (cram-tf:ensure-pose-in-frame pose cram-tf:*fixed-frame*))

(defun ensure-base-goal-reached (goal-pose)
  (unless (cram-tf:tf-frame-converged
           cram-tf:*robot-base-frame* goal-pose
           *base-convergence-delta-xy* *base-convergence-delta-theta*)
    (make-instance 'common-fail:navigation-goal-not-reached
      :description (format nil "Giskard did not converge to goal:~%~
                                ~a should have been at ~a ~
                                with delta-xy of ~a and delta-angle of ~a."
                           cram-tf:*robot-base-frame* goal-pose
                           *base-convergence-delta-xy*
                           *base-convergence-delta-theta*))))

(defun call-base-action (&key action-timeout goal-pose)
  (declare (type cl-transforms-stamped:pose-stamped goal-pose)
           (type (or null number) action-timeout))

  (setf goal-pose (ensure-base-goal-input goal-pose))

  (cram-tf:visualize-marker goal-pose :r-g-b-list '(0 1 0))

  (call-action
   :action-goal (make-giskard-base-action-goal goal-pose)
   :action-timeout action-timeout
   :check-goal-function (lambda () (ensure-base-goal-reached goal-pose))))
