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

(in-package :pr2-ll)

(defparameter *giskard-action-timeout* 10.0
  "How many seconds to wait before returning from giskard action.")

(defvar *giskard-action-client* nil)

(defun init-giskard-action-client ()
  (setf *giskard-action-client* (actionlib:make-action-client
                                 "controller_action_server/move"
                                 "giskard_msgs/WholeBodyAction"))
  (loop until (actionlib:wait-for-server *giskard-action-client* 5.0)
        do (roslisp:ros-info (giskard-action-client) "Waiting for giskard action server..."))
  (roslisp:ros-info (giskard-action-client) "giskard action client created.")
  *giskard-action-client*)

(defun destroy-giskard-action-client ()
  (setf *giskard-action-client* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-giskard-action-client)

(defun get-giskard-action-client ()
  (or *giskard-action-client*
      (init-giskard-action-client)))

(defun make-giskard-action-goal (frame left-pose right-pose)
  ;; (declare (type (or null cl-transforms:pose) left-pose right-pose)
  ;;          (type string frame))
  (let ((left-pose-msg (cl-transforms-stamped:to-msg
                        (cl-transforms-stamped:ensure-pose-stamped
                         (or left-pose
                             (cl-transforms:make-identity-pose))
                         frame 0.0)))
        (right-pose-msg (cl-transforms-stamped:to-msg
                         (cl-transforms-stamped:ensure-pose-stamped
                          (or right-pose
                              (cl-transforms:make-identity-pose))
                          frame 0.0))))
    (actionlib:make-action-goal
        (get-giskard-action-client)
      (goal left_ee command) left-pose-msg
      (process left_ee command) left-pose
      (goal right_ee command) right-pose-msg
      (process right_ee command) right-pose)))

(define-condition giskard-action-failed (pr2-low-level-failure) ())

(defun call-giskard-action (left-pose right-pose
                            &optional (ik-base-frame *robot-base-frame*))
  (multiple-value-bind (result status)
      (cpl:with-failure-handling
          ((simple-error (e)
             (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
             (init-giskard-action-client)
             (cpl:retry)))
        (let ((actionlib:*action-server-timeout* 10.0))
          (actionlib:call-goal
           (get-giskard-action-client)
           (make-giskard-action-goal ik-base-frame left-pose right-pose)
           :timeout *giskard-action-timeout*)))
    (unless (eql status :succeeded)
      (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
    (values result status)))
