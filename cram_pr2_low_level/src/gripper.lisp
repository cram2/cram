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

(defvar *gripper-action-clients* '(:left nil :right nil)
  "A list to store PR2 gripper action clients for left and right gripper.")

(defun init-gripper-action-client (left-or-right)
  (let ((action-server-name (concatenate
                             'string
                             (ecase left-or-right
                               (:left "l")
                               (:right "r"))
                             "_gripper_controller/gripper_action")))
    (setf (getf *gripper-action-clients* left-or-right)
          (actionlib:make-action-client
           action-server-name
           "pr2_controllers_msgs/Pr2GripperCommandAction"))
    (loop until (actionlib:wait-for-server
                 (getf *gripper-action-clients* left-or-right)
                 5.0)
          do (roslisp:ros-info (gripper-action)
                               "Waiting for ~a gripper action server."
                               left-or-right))
    (roslisp:ros-info (gripper-action)
                      "~a gripper action client created."
                      left-or-right)))

(defun init-gripper-action-clients ()
  (mapc #'init-gripper-action-client '(:left :right)))

(defun destroy-gripper-action-clients ()
  (setf *gripper-action-clients* '(:left nil :right nil)))

(roslisp-utilities:register-ros-cleanup-function destroy-gripper-action-clients)


(defun get-gripper-action-client (left-or-right)
  (or (getf *gripper-action-clients* left-or-right)
      (init-gripper-action-client left-or-right)))

(defun make-gripper-action-goal (action-client position &optional max-effort)
  (actionlib:make-action-goal action-client
    (position command) (etypecase position
                         (number (cond
                                   ((< position 0.0)
                                    (roslisp:ros-warn
                                     (gripper-action)
                                     "POSITION (~a) cannot be smaller than 0.0. Clipping."
                                     position)
                                    0.0)
                                   ((> position 0.085)
                                    (roslisp:ros-warn
                                     (gripper-action)
                                     "POSITION (~a) shouldn't be bigger than 0.085. Clipping."
                                     position)
                                    0.085)
                                   (t
                                    position)))
                         (keyword (ecase position
                                    (:open 0.085)
                                    (:close 0.0)
                                    (:grip 0.0))))
    (max_effort command) (or max-effort (etypecase position
                                          (number 50.0)
                                          (keyword (ecase position
                                                     (:open -1)
                                                     (:close 500.0) ; because of stupid gazebo
                                                     (:grip 50.0)))))))

(defun call-gripper-action (left-or-right position &optional max-effort)
  "`position' can be :open, :close, :grip or a joint position."
  (multiple-value-bind (result status)
      (cpl:with-failure-handling
          ((simple-error (e)
             (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
             (init-gripper-action-clients)
             (cpl:retry)))
        (let ((actionlib:*action-server-timeout* 10.0)
              (action-client (get-gripper-action-client left-or-right)))
          (actionlib:call-goal
           action-client
           (make-gripper-action-goal action-client position max-effort)
           :timeout 10.0)))
    (if (eql position :grip) ; gripper should stall and action should result :aborted
        (unless (eql status :aborted)
          (cpl:fail 'cram-plan-failures:gripping-failed))
        (unless (eql status :succeeded)
          (cpl:fail 'cram-plan-failures:gripping-failed)))
    (values result status)))
