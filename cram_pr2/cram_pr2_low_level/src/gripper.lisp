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

(defparameter *gripper-convergence-delta* 0.001 "in meters")

(defparameter *gripper-action-timeout* 10.0 "in seconds")

(defparameter *gripper-minimal-position* 0.0017235310878717042d0 "in meters")

(defvar *gripper-action-clients* '(:left nil :right nil)
  "A list to store PR2 gripper action clients for left and right gripper.")

(defun init-gripper-action-client (left-or-right)
  (let ((action-server-name (concatenate
                             'string
                             (ecase left-or-right
                               (:left "l")
                               (:right "r"))
                             "_gripper_controller/gripper_action")))
    (prog1
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
                        left-or-right))))

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
    (position command) position
    (max_effort command) max-effort))

(defun ensure-gripper-input-parameters (position max-effort)
  (let ((position
          (etypecase position
            (number (cond
                      ((< position 0.0)
                       (roslisp:ros-warn (gripper-action)
                                         "POSITION (~a) cannot be < 0.0. Clipping."
                                         position)
                       0.0)
                      ((> position 0.085)
                       (roslisp:ros-warn (gripper-action)
                                         "POSITION (~a) shouldn't be > 0.085. Clipping."
                                         position)
                       0.085)
                      (t
                       position)))
            (keyword (ecase position
                       (:open 0.085)
                       (:close *gripper-minimal-position*)
                       (:grip *gripper-minimal-position*)))))
        (max-effort
          (or max-effort
              (etypecase position
                (number 50.0)
                (keyword (ecase position
                           (:open -1)
                           (:close 500.0) ; because of stupid gazebo
                           (:grip 50.0)))))))
    (values position max-effort)))


(defun ensure-gripper-goal-reached (result
                                    status
                                    original-goal-position
                                    goal-position
                                    convergence-delta)
  (when (eql status :timeout)
    (roslisp:ros-warn (pr2-ll gripper) "Gripper action timed out."))
  (let* ((current-position
           (pr2_controllers_msgs-msg:position result))
         ;; TODO: use current-position from joint state message, not result
         ;; (current-position (car (joints:joint-positions (list cram-tf:*left-gripper-joint*))))
         (converged-p
           (cram-tf:values-converged current-position goal-position convergence-delta)))
    (if (eql original-goal-position :grip) ; gripper should not completely close
        (when converged-p
          (cpl:fail 'common-fail:gripper-closed-completely
                    :description "Tried to grasp but ended up closing the gripper."))
        (unless converged-p
          (cpl:fail 'common-fail:gripper-goal-not-reached
                    :description (format nil "Gripper action did not converge to the goal:
goal: ~a, current: ~a, delta: ~a." goal-position current-position convergence-delta))))))

(defun call-gripper-action (left-or-right position &key
                                                     max-effort
                                                     (delta *gripper-convergence-delta*)
                                                     (action-timeout *gripper-action-timeout*))
  (declare (type (or keyword list) left-or-right)
           (type (or keyword number) position)
           (type (or null number) max-effort)
           (type number delta action-timeout))
  "`goal-position' can be :open, :close, :grip or a joint position."
  (multiple-value-bind (goal-position max-effort)
       (ensure-gripper-input-parameters position max-effort)
    (flet ((move-the-hand-yo (l-or-r)
             (multiple-value-bind (result status)
                 (cpl:with-failure-handling
                     ((simple-error (e)
                        (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
                        (init-gripper-action-clients)
                        (cpl:retry)))
                   (let ((actionlib:*action-server-timeout* 10.0)
                         (action-client (get-gripper-action-client l-or-r)))
                     (if (eql position :grip) ; double check that we really grasped properly
                         (progn
                           (actionlib:call-goal
                            action-client
                            (make-gripper-action-goal action-client goal-position max-effort)
                            :timeout action-timeout)
                           (cpl:sleep 0.5)
                           (actionlib:call-goal
                            action-client
                            (make-gripper-action-goal action-client goal-position max-effort)
                            :timeout action-timeout))
                         (actionlib:call-goal
                          action-client
                          (make-gripper-action-goal action-client goal-position max-effort)
                          :timeout action-timeout))))
               (ensure-gripper-goal-reached result status position goal-position delta)
               (values result status))))
      (if (and left-or-right (listp left-or-right))
          (cpl:par
            (move-the-hand-yo (first left-or-right))
            (move-the-hand-yo (second left-or-right)))
          (move-the-hand-yo left-or-right)))))
