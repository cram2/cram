;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defun make-giskard-action-client ()
  (actionlib-client:make-simple-action-client
   'giskard-action
   "giskard/command" "giskard_msgs/MoveAction"
   120))

(roslisp-utilities:register-ros-init-function make-giskard-action-client)

(defun call-action (&key action-goal action-timeout check-goal-function)
  (declare (type giskard_msgs-msg:movegoal action-goal)
           (type (or number null) action-timeout)
           (type (or function null) check-goal-function))

  ;; check if the goal has already been reached
  (when (and check-goal-function
             (not (funcall check-goal-function nil nil)))
    (roslisp:ros-warn (giskard action-client)
                      "Giskard action goal already reached.")
    (return-from call-action))

  ;; call the actionlib action
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'giskard-action
       :action-goal action-goal
       :action-timeout action-timeout)

    ;; print a debug statement if the status is unexpected
    (case status
      (:preempted
       (roslisp:ros-warn (giskard action-client)
                         "Giskard action preempted.~%Result: ~a" result))
      (:timeout
       (roslisp:ros-warn (giskard action-client)
                         "Giskard action timed out."))
      (:aborted
       (roslisp:ros-warn (giskard cartesian)
                         "Giskard action aborted.~%Result: ~a" result)))

    (when (and result
               (member (roslisp:symbol-code
                        'giskard_msgs-msg:moveresult
                        :unknown_group)
                       (map 'list #'identity
                            (roslisp:msg-slot-value
                             result
                             :error_codes))))
      (full-update-collision-scene))

    ;; check if the goal was reached, if not, throw a failure
    (when check-goal-function
      (let ((failure (funcall check-goal-function result status)))
        (when failure
          (roslisp:ros-warn (giskard action-client)
                            "Giskard action goal was not reached.")
          (cpl:fail failure))))

    ;; this is only used by HPN:
    ;; return the joint state, which is our observation
    ;; (joints:full-joint-states-as-hash-table)

    ;; return the result and status
    (values result status)))
