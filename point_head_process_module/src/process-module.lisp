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

(in-package :point-head-process-module)

(defvar *action-client* nil)
(defvar *point-head-thread* nil)

(defun init-point-head-action ()
  (setf *action-client* (actionlib:make-action-client "/head_traj_controller/point_head_action"
                                                      "pr2_controllers_msgs/PointHeadAction")))

(register-ros-init-function init-point-head-action)

(def-process-module point-head-process-module (goal)
  (handler-case
      (destructuring-bind (cmd action-goal) (reference goal)
        (maybe-shutdown-thread)
        (ecase cmd
          (point
             (actionlib:send-goal-and-wait
              *action-client* action-goal
              :result-timeout 1.0
              :exec-timeout 3.0))
          (follow
             (actionlib:send-goal-and-wait
              *action-client* action-goal
              :result-timeout 1.0
              :exec-timeout 3.0)
             ;; (setf *point-head-thread*
             ;;       (sb-thread:make-thread
             ;;        (curry #'follow-pose-thread-fun action-goal)))
             )))
    ;; Ugly hack. We shouldn't catch errors here but find a way to
    ;; resolve all designators.
    (designator-error (e)
      (declare (ignore e))
      (roslisp:ros-warn (point-head process-module) "Cannot resolve designator ~a. Ignoring."
                        goal))))

(defun maybe-shutdown-thread ()
  (when (and *point-head-thread*
             (sb-thread:thread-alive-p *point-head-thread*))
    (sb-thread:terminate-thread *point-head-thread*)
    (setf *point-head-thread* nil)))

(defun follow-pose-thread-fun (goal)
  (top-level
    (roslisp-utils:loop-at-most-every 0.01
      (pursue
        (actionlib:call-goal *action-client* goal)
        (sleep 1)))))

(defmethod pm-run :around ((pm point-head-process-module))
  (unwind-protect
       (call-next-method)
    (when (and *point-head-thread*
               (sb-thread:thread-alive-p *point-head-thread*))
      (sb-thread:terminate-thread *point-head-thread*))))
