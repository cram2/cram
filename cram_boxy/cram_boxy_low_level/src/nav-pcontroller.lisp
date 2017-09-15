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

(in-package :boxy-ll)

(defvar *xy-goal-tolerance* 0.05 "in meters")
(defvar *yaw-goal-tolerance* 0.1 "in radiants, about 6 degrees.")
;; There are VARs (not PARAMETERs) as they're read from the ROS param server

(defparameter *nav-p-timeout* 20.0 "in seconds")

(defun init-nav-pcontroller-action-client-and-read-params ()
  (make-simple-action-client
   'nav-pcontroller-action
   "nav_pcontroller/move_base" "move_base_msgs/MoveBaseAction"
   60.0)

  (setf *xy-goal-tolerance*
        (roslisp:get-param "nav_pcontroller/xy_tolerance" *xy-goal-tolerance*))
  (setf *yaw-goal-tolerance*
        (roslisp:get-param "nav_pcontroller/th_tolerance" *yaw-goal-tolerance*)))

(roslisp-utilities:register-ros-init-function init-nav-pcontroller-action-client-and-read-params)

(defun make-nav-p-action-goal (pose)
  (roslisp:make-message
   'move_base_msgs-msg:MoveBaseGoal
    :target_pose (cl-transforms-stamped:to-msg pose)))

(defun ensure-nav-p-goal-reached (status goal-pose convergence-delta-xy convergence-delta-theta)
  (when (eql status :timeout)
    (cpl:fail 'common-fail:navigation-pose-unreachable
              :description "Nav-pcontroller action timed out"))
  (unless (cram-tf:tf-frame-converged cram-tf:*robot-base-frame* goal-pose
                                      convergence-delta-xy convergence-delta-theta)
    (cpl:fail 'common-fail:navigation-low-level-failure
              :description (format nil "Nav-pcontroller did not converge to goal:
~a should have been at ~a with delta-xy of ~a and delta-angle of ~a."
                                   cram-tf:*robot-base-frame* goal-pose
                                   convergence-delta-xy convergence-delta-theta))))

(defun move-base-nav-pcontroller (&key goal-pose
                                    (action-timeout *nav-p-timeout*)
                                    (convergence-delta-xy *xy-goal-tolerance*)
                                    (convergence-delta-theta *yaw-goal-tolerance*)
                                    visualize)
  (declare (type (or null cl-transforms-stamped:pose-stamped) goal-pose)
           (type (or null number) action-timeout convergence-delta-xy convergence-delta-theta)
           (type boolean visualize))
  (let ((goal-pose (cram-tf:ensure-pose-in-frame goal-pose cram-tf:*fixed-frame*)))
    (when visualize
      (visualize-marker goal-pose :topic "low-level-goals"))
    (multiple-value-bind (result status)
        (call-simple-action-client
         'nav-pcontroller-action
         :action-goal (make-nav-p-action-goal goal-pose)
         :action-timeout action-timeout)
      (roslisp:ros-info (navigation low-level) "Nav action finished.")
      (ensure-nav-p-goal-reached
       status goal-pose convergence-delta-xy convergence-delta-theta)
      (values result status))))
