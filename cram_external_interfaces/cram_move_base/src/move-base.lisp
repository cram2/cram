;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :move-base)

;;;;;;;;;;;;;;;;;;;;;;
;; Actionlib Client ;;
(defun make-move-base-action-client ()
  (actionlib-client:make-simple-action-client
   'move-base-action
   "move_base" "move_base_msgs/MoveBaseAction"
   120))

(roslisp-utilities:register-ros-init-function make-move-base-action-client)

(defun call-action (&key action-goal action-timeout check-goal-function)
  (declare (type move_base_msgs-msg:movebasegoal action-goal)
           (type (or number null) action-timeout)
           (type (or function null) check-goal-function))

  ;; check if the goal has already been reached
  (when (and check-goal-function
             (not (funcall check-goal-function nil nil)))
    (roslisp:ros-warn (move-base action-client)
                      "Move-Base action goal already reached.")
    (return-from call-action))

  ;; call the actionlib action
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'move-base-action
       :action-goal action-goal
       :action-timeout action-timeout)

    ;; print a debug statement if the status is unexpected
    (case status
      (:preempted
       (roslisp:ros-warn (move-base action-client)
                         "Move-base action preempted.~%Result: ~a" result))
      (:timeout
       (roslisp:ros-warn (move-base action-client)
                         "Move-base action timed out."))
      (:aborted
       (roslisp:ros-warn (move-base cartesian)
                         "Move-base action aborted.~%Result: ~a" result)))

    ;; check if the goal was reached, if not, throw a failure
    (when check-goal-function
      (let ((failure (funcall check-goal-function result status)))
        (when failure
          (roslisp:ros-warn (move-base action-client)
                            "Move-base action goal was not reached.")
          (cpl:fail failure))))

    ;; return the result and status
    (values result status)))
;; Actionlib Client ;;
;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;
;; Move-Base interface ;;
(defparameter *base-convergence-delta-xy* 0.1
  "in meters.
  Adjust this to the xy_goal_tolerance in the move_base_params config.")
(defparameter *base-convergence-delta-theta* 0.1
  "in radiants, about 6 degrees.
  Adjust this to the yaw_goal_tolerance in the move_base_params config.")

(defun make-move-base-action-goal (pose)
  (declare (type cl-transforms-stamped:pose-stamped pose)
           (type (or keyword number null) base-velocity))
  (roslisp:make-message
   'move_base_msgs-msg:MoveBaseGoal
   :target_pose (cl-transforms-stamped:to-msg pose)))

(defun ensure-base-goal-input (pose)
  (cram-tf:ensure-pose-in-frame pose cram-tf:*fixed-frame*))

(defun ensure-base-goal-reached (goal-pose)
  (unless (cram-tf:tf-frame-converged
           cram-tf:*robot-base-frame* goal-pose
           *base-convergence-delta-xy* *base-convergence-delta-theta*)
    (make-instance 'common-fail:navigation-goal-not-reached
      :description (format nil "Move-base did not converge to goal:~%~
                                ~a should have been at ~a ~
                                with delta-xy of ~a and delta-angle of ~a."
                           cram-tf:*robot-base-frame* goal-pose
                           *base-convergence-delta-xy*
                           *base-convergence-delta-theta*))))

(defun call-base-action (&key action-timeout goal-pose)
  (declare (type cl-transforms-stamped:pose-stamped goal-pose)
           (type (or null number) action-timeout))
  (setf goal-pose (ensure-base-goal-input goal-pose))
  (call-action
   :action-goal (make-move-base-action-goal goal-pose)
   :action-timeout action-timeout
   :check-goal-function (lambda (result status)
                          (declare (ignore result status))
                          (ensure-base-goal-reached goal-pose))))
;; Move-Base interface ;;
;;;;;;;;;;;;;;;;;;;;;;;;;
