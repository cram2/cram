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

(defvar *nav-pcontroller-action-client* nil
  "Actionlib client for nav-pcontroller.")

(defvar *xy-goal-tolerance* 0.05 "in meters")
(defvar *yaw-goal-tolerance* 0.1 "in radiants, about 6 degrees.")
;; There are VARs (not PARAMETERs) as they're read from the ROS param server

(defun init-nav-pcontroller ()
  (setf *nav-pcontroller-action-client*
        (actionlib:make-action-client
         "/nav_pcontroller/move_base" "move_base_msgs/MoveBaseAction"))
  (loop until (actionlib:wait-for-server *nav-pcontroller-action-client* 5.0)
        do (roslisp:ros-info (navigation low-level) "Waiting for nav_pcontroller server..."))
  (roslisp:ros-info (navigation low-level) "nav_pcontroller action client created.")

  (when (roslisp:has-param "/nav_pcontroller/xy_tolerance")
    (setf *xy-goal-tolerance* (roslisp:get-param "/nav_pcontroller/xy_tolerance")))
  (when (roslisp:has-param "/nav_pcontroller/th_tolerance")
    (setf *yaw-goal-tolerance* (roslisp:get-param "/nav_pcontroller/th_tolerance")))

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

(defun goal-reached-p (goal-pose)
  (let* ((pose-in-base (cl-transforms-stamped:transform-pose-stamped
                        *transformer*
                        :pose goal-pose :target-frame *robot-base-frame*
                        :timeout *tf-default-timeout*))
         (goal-dist (max (abs (cl-transforms:x (cl-transforms:origin pose-in-base)))
                         (abs (cl-transforms:y (cl-transforms:origin pose-in-base)))))
         (goal-angle (cl-transforms:normalize-angle
                      (cl-transforms:get-yaw
                       (cl-transforms:orientation pose-in-base)))))
    (if (or (> goal-dist *xy-goal-tolerance*)
            (> (abs goal-angle) *yaw-goal-tolerance*))
        (progn
          (roslisp:ros-warn
           (navigation low-level)
           "Goal not reached. Linear distance: ~a, angular distance: ~a"
           goal-dist goal-angle)
          nil)
        t)))

(defun call-nav-pcontroller-action (goal-pose)
  (let ((goal-pose-in-fixed-frame
          (cl-transforms-stamped:transform-pose-stamped
           *transformer*
           :pose goal-pose
           :target-frame *fixed-frame*
           :timeout *tf-default-timeout*
           :use-current-ros-time t)))
    ;; todo publish nav goals
    (roslisp:publish (roslisp:advertise "/ppp" "geometry_msgs/PoseStamped")
                     (cl-transforms-stamped:to-msg goal-pose-in-fixed-frame))
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
             :timeout 60.0)))
      (roslisp:ros-info (navigation low-level) "Nav action finished.")
      (unless (goal-reached-p (cl-transforms-stamped:copy-pose-stamped
                               goal-pose-in-fixed-frame :stamp 0))
        (error "OMG")
        ;; (cpl:fail 'location-not-reached-failure
        ;;           :location goal-pose)
        ))))

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

