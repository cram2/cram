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

(defvar *point-head-thread* nil)

(defun init-point-head-action ()
  (setf *action-client* (actionlib-lisp:make-simple-action-client
                         "/head_traj_controller/point_head_action"
                         "pr2_controllers_msgs/PointHeadAction")))

(roslisp-utilities:register-ros-init-function init-point-head-action)

(cut:define-hook cram-language::on-begin-move-head (pose-stamped))
(cut:define-hook cram-language::on-finish-move-head (id success))

(def-process-module point-head-process-module (goal)
  (let ((log-id (first (cram-language::on-begin-move-head (reference goal))))
        (success nil))
    (unwind-protect
         (handler-case
             (destructuring-bind (cmd action-goal) (reference goal)
               (maybe-shutdown-thread)
               (actionlib-lisp:wait-for-server *action-client*)
               (ecase cmd
                 (point
                  (actionlib-lisp:send-goal-and-wait
                   *action-client* action-goal
                   1.0 3.0))
                 (follow
                  (actionlib-lisp:send-goal-and-wait
                   *action-client* action-goal
                   10.0 10.0)
                  ;; (setf *point-head-thread*
                  ;;       (sb-thread:make-thread
                  ;;        (curry #'follow-pose-thread-fun action-goal)))
                  ))
               (setf success t))
           ;; Ugly hack. We shouldn't catch errors here but find a way to
           ;; resolve all designators.
           (designator-error (e)
             (declare (ignore e))
             (roslisp:ros-warn
              (point-head process-module)
              "Cannot resolve designator ~a. Ignoring." goal)))
      (cram-language::on-finish-move-head log-id success)
      (cram-occasions-events:on-event
       (make-instance 'cram-plan-occasions-events:robot-state-changed
                      :timestamp 0.0)))))

(defun maybe-shutdown-thread ()
  (when (and *point-head-thread*
             (sb-thread:thread-alive-p *point-head-thread*))
    (sb-thread:terminate-thread *point-head-thread*)
    (setf *point-head-thread* nil)))

(defun follow-pose-thread-fun (goal)
  (top-level
    (roslisp-utils:loop-at-most-every 0.01
      (pursue
        ;; This is commented out to make sure that we don't
        ;; accidentally trigger this action through some side
        ;; effect. The head following a pose is a neat feature and
        ;; should be re-introduced, but doesn't work right now - so we
        ;; leave its function infrastructure here and fix it later.
        ;(actionlib-lisp:call-goal *action-client* goal)
        (cpl:sleep 1)))))

(defmethod pm-run :around ((pm point-head-process-module) &optional name)
  (declare (ignore name))
  (unwind-protect
       (call-next-method)
    (when (and *point-head-thread*
               (sb-thread:thread-alive-p *point-head-thread*))
      (sb-thread:terminate-thread *point-head-thread*))))
