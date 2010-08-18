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

(in-package :kipla)

(define-condition navigation-failure (plan-error)
  ((location :initarg :location :initform nil :reader navigation-failure-location)))

(define-condition location-not-reached-failure (navigation-failure) ())

(define-condition location-reached-but-not-terminated (plan-error) ())

(defun get-nav-waypoints (goal)
  (list goal))

(defun approach-waypoint (goal last? &optional (threshold 0.3))
  (setf (value *navigation-distance-to-goal-fluent*) 100)
  (if last?
      (navigation-execute-goal (jlo:id goal))
      (pursue
        (navigation-execute-goal (jlo:id goal))
        (wait-for (< *navigation-distance-to-goal-fluent* threshold)))))

(def-process-module navigation (input)
  ;; This process module navigates the robot to the location given as
  ;; input.
  (unless (member :navigation *kipla-features*)
    (sleep 0.1)
    (return nil))
  (log-msg :info "[Navigation process module] received input ~a~%" (reference input))
  (let ((waypoints (get-nav-waypoints (pose->jlo (reference input)))))
    (log-msg :info "[Navigation process module] waypoints `~a'" waypoints)
    (loop while waypoints
          do
       (log-msg :info "Drive to waypoint `~a'" (car waypoints))
       (approach-waypoint (car waypoints) (not (cdr waypoints)))
       (sleep 0.1)
       (setf waypoints (cdr waypoints))))
  (log-msg :info "[Navigation process module] returning.~%"))
