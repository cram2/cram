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

(in-package :pr2-ll)

(defvar *xy-goal-tolerance* 0.05 "in meters")
(defvar *yaw-goal-tolerance* 0.1 "in radiants, about 6 degrees.")
;; There are VARs (not PARAMETERs) as they're read from the ROS param server

(defparameter *navigation-action-timeout* 60.0 "in seconds")

(defvar *nav-pcontroller-action-client* nil
  "Actionlib client for nav-pcontroller.")

(defun init-nav-pcontroller ()
  (setf *nav-pcontroller-action-client*
        (actionlib:make-action-client
         "/nav_pcontroller/move_base" "move_base_msgs/MoveBaseAction"))
  (loop until (actionlib:wait-for-server *nav-pcontroller-action-client* 5.0)
        do (roslisp:ros-info (navigation low-level) "Waiting for nav_pcontroller server..."))
  (roslisp:ros-info (navigation low-level) "nav_pcontroller action client created.")

  (when (roslisp:has-param "/nav_pcontroller/xy_tolerance")
    (setf *xy-goal-tolerance* (+ 0.01 (roslisp:get-param "/nav_pcontroller/xy_tolerance"))))
  (when (roslisp:has-param "/nav_pcontroller/th_tolerance")
    (setf *yaw-goal-tolerance* (+ 0.01 (roslisp:get-param "/nav_pcontroller/th_tolerance"))))

  *nav-pcontroller-action-client*)

(defun destroy-nav-pcontroller ()
  (setf *nav-pcontroller-action-client* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-nav-pcontroller)

(defun get-nav-pcontroller-action-client ()
  (or *nav-pcontroller-action-client*
      (init-nav-pcontroller)))

(defun make-nav-p-action-goal (pose)
  (actionlib:make-action-goal
      (get-nav-pcontroller-action-client)
    :target_pose (cl-transforms-stamped:to-msg pose)))

(defun ensure-nav-p-goal-reached (status goal-pose convergence-delta-xy convergence-delta-theta)
  (when (eql status :timeout)
    (cpl:fail 'common-fail:actionlib-action-timed-out
              :description "Nav-pcontroller action timed out"))
  (unless (cram-tf:tf-frame-converged cram-tf:*robot-base-frame* goal-pose
                              convergence-delta-xy convergence-delta-theta)
    (cpl:fail 'common-fail:navigation-low-level-failure
              :description (format nil "Nav-pcontroller did not converge to goal:
~a should have been at ~a with delta-xy of ~a and delta-angle of ~a."
                                   cram-tf:*robot-base-frame* goal-pose
                                   convergence-delta-xy convergence-delta-theta))))

(defun call-nav-pcontroller-action (goal-pose &key
                                                (convergence-delta-xy *xy-goal-tolerance*)
                                                (convergence-delta-theta *yaw-goal-tolerance*)
                                                (action-timeout *navigation-action-timeout*)
                                                visualize)
  (declare (type (or cl-transforms:pose cl-transforms-stamped:pose-stamped))
           (type number convergence-delta-xy convergence-delta-theta action-timeout))
  "If `goal-pose' is a CL-TRANSFORMS:POSE it's in *fixed-frame*."
  (let ((goal-pose-in-fixed-frame
          (cram-tf:ensure-pose-in-frame goal-pose cram-tf:*fixed-frame*)))
    (when visualize
      (visualize-marker goal-pose :topic "low-level-goals"))
    (multiple-value-bind (result status)
        (cpl:with-failure-handling
            ((simple-error (e)
               (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
               (init-nav-pcontroller)
               (cpl:retry)))
          (let ((actionlib:*action-server-timeout* 10.0))
            (actionlib:call-goal
             (get-nav-pcontroller-action-client)
             (make-nav-p-action-goal goal-pose-in-fixed-frame)
             :timeout action-timeout)))
      (roslisp:ros-info (navigation low-level) "Nav action finished.")
      (ensure-nav-p-goal-reached
       status goal-pose-in-fixed-frame convergence-delta-xy convergence-delta-theta)
      (values result status))))

;; (def-process-module pr2-navigation-process-module (goal)
;;   (unwind-protect
;;        (progn
;;          (roslisp:ros-info (pr2-nav process-module) "Using nav-pcontroller.")
;;          (let* ((goal-location-designator (reference goal))
;;                 (goal-pose (reference goal-location-designator)))
;;           (call-nav-pcontroller-action goal-pose)))
;;     (roslisp:ros-info (pr2-nav process-module) "Navigation finished.")
;;     (cram-occasions-events:on-event
;;      (make-instance 'cram-plan-occasions-events:robot-state-changed))))

