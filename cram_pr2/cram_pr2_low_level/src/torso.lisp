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

(defvar *torso-action-client* nil)

(defparameter *torso-action-timeout* 20.0 "in seconds")
(defparameter *torso-convergence-delta* 0.005 "in meters")

(defun init-torso-action-client ()
  (prog1
      (setf *torso-action-client* (actionlib:make-action-client
                                   "torso_controller/position_joint_action"
                                   "pr2_controllers_msgs/SingleJointPositionAction"))
    (loop until (actionlib:wait-for-server *torso-action-client* 5.0)
          do (roslisp:ros-info (torso-action-client)
                               "Waiting for torso controller actionlib server..."))
    (roslisp:ros-info (torso-action-client) "Torso shape action client created.")))

(defun destroy-torso-action-client ()
  (setf *torso-action-client* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-torso-action-client)

(defun get-torso-action-client ()
  (or *torso-action-client*
      (init-torso-action-client)))

(defun make-torso-action-goal (position)
  (actionlib:make-action-goal
      (get-torso-action-client)
    :position position))

(defun ensure-torso-input-parameters (position)
  (declare (type (or keyword number) position))
  (etypecase position
    (number (cond
              ((< position 0.02)
               (roslisp:ros-warn
                (torso-action)
                "POSITION (~a) cannot be smaller than 0.02. Clipping."
                position)
               0.02)
              ((> position 0.32)
               (roslisp:ros-warn
                (torso-action)
                "POSITION (~a) shouldn't be bigger than 0.32. Clipping."
                position)
               0.32)
              (t
               position)))
    (keyword (ecase position
               (:up 0.32)
               (:down 0.02)))))

(defun ensure-torso-goal-reached (status goal-position convergence-delta)
  (when (eql status :timeout)
    (cpl:fail 'common-fail:actionlib-action-timed-out :description "Torso action timed out"))
  (let ((current-position (car (joint-positions (list cram-tf:*robot-torso-joint*)))))
   (unless (values-converged current-position goal-position convergence-delta)
     (cpl:fail 'common-fail:low-level-failure
               :description (format nil "Torso action did not converge to the goal:
goal: ~a, current: ~a, delta: ~a." goal-position current-position convergence-delta)))))

(defun call-torso-action (position &key
                                     (action-timeout *torso-action-timeout*)
                                     (convergence-delta *torso-convergence-delta*))
  (declare (type (or keyword number) position)
           (type number action-timeout convergence-delta))
  (let ((position-number (ensure-torso-input-parameters position)))
   (multiple-value-bind (result status)
       (cpl:with-failure-handling
           ((simple-error (e)
              (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
              (init-torso-action-client)
              (cpl:retry)))
         (let ((actionlib:*action-server-timeout* 10.0))
           (actionlib:call-goal
            (get-torso-action-client)
            (make-torso-action-goal position-number)
            :timeout action-timeout)))
     (roslisp:ros-info (torso-action-client) "Torso action finished.")
     (ensure-torso-goal-reached status position convergence-delta)
     (values result status))))






;; (defconstant +max-torso-joint-position+ 0.33)
;; (defconstant +min-torso-joint-position+ 0.0)

;; (defun make-torso-action-goal-cpl (position)
;;   (actionlib:make-action-goal *torso-action-client*
;;     position (etypecase position
;;                (number position)
;;                (keyword (ecase position
;;                           (:up +max-torso-joint-position+)
;;                           (:down +min-torso-joint-position+))))))

;; ;;; TODO: check if it's still stuck at the same spot after 5 seconds
;; (defparameter *position-delta* 0.001)
;; (defun abort-when-small-improvement (feedback-signal)
;;   (setf *last-torso-position* feedback-signal)
;;   (with-slots (actionlib:feedback) feedback-signal
;;     (roslisp:with-fields (position (stamp (stamp header)))
;;         actionlib:feedback
;;       (let ((position-some-time-ago stamp)) ;; it's f(stamp) not stamp
;;         (when (< (abs (- position position-some-time-ago)) *position-delta*)
;;                 (roslisp:ros-warn (torso-action)
;;                                   "Torso action not converging. Aborting.")
;;                 (invoke-restart 'actionlib:abort-goal))))))

;; (defun call-torso-action-cpl (position &key (timeout 60.0))
;;   "Per default this action should take no longer than one minute."
;;   (multiple-value-bind (result status)
;;       (cpl:with-failure-handling
;;           ((simple-error (e)
;;              (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
;;              (init-torso-action-client)
;;              (cpl:retry)))
;;         (let ((actionlib:*action-server-timeout* 10.0))
;;           (handler-bind
;;               ((actionlib:feedback-signal #'abort-when-small-improvement))
;;             (actionlib:call-goal
;;              *torso-action-client*
;;              (make-torso-action-goal position)
;;              :timeout timeout))))
;;     (roslisp:ros-info (torso-action-client) "Torso action finished.")
;;     (values result status)))
