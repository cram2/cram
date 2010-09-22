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

(in-package :navp-pm)

(define-condition nav-action-error (simple-plan-error)
  ((final-status :initarg :final-status :reader final-status)))

(defconstant +min-nav-distance-threshold+ 0.05)

(defvar *navigation-speed-fluent*
  (make-fluent :name 'navigation-speed-fluent :value 0))
(defvar *navigation-distance-to-goal-fluent*
  (make-fluent :name 'navigation-distance-to-goal-fluent :value 0))
(defvar *nav-action-client* nil)

(defun nav-action-init ()
  (setf *nav-action-client* (actionlib:make-action-client
                             "/nav_pcontroller/nav_action"
                             "navp_action/nav_actionAction")))

(register-ros-init-function nav-action-init)

(defun navigation-execute-goal (lo-id)
  (ros-info (nav process-module) "executing nav action")
  (multiple-value-bind (result state)
      (actionlib:call-goal *nav-action-client*
                           (make-message "navp_action/nav_actionGoal" (data target_lo) lo-id)
                           :feedback-cb (lambda (x)
                                          (setf (value *navigation-speed-fluent*)
                                                (std_msgs-msg:data-val
                                                 (navp_action-msg:speed-val x)))
                                          (setf (value *navigation-distance-to-goal-fluent*)
                                                (std_msgs-msg:data-val
                                                 (navp_action-msg:distance-val x)))))
    (when (and (not (eq state :succeeded))
               (> (std_msgs-msg:data-val (navp_action-msg:distance-val result))
                  +min-nav-distance-threshold+))
      (error 'nav-action-error
             :format-control "Navigation action failed: ~a."
             :format-arguments (list result)
             :final-status state))
    result))
