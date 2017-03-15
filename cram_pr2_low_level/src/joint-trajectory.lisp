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

(defvar *joint-trajectory-action-clients* '(:left nil :right nil)
  "A list to store PR2 arm trajectory action clients for left and right arm.")

(defparameter *max-joint-velocity* 0.5 "in radians/sec")

(defun init-joint-trajectory-action-client (left-or-right)
  (let ((action-server-name (concatenate
                             'string
                             (ecase left-or-right
                               (:left "l")
                               (:right "r"))
                             "_arm_controller/joint_trajectory_action")))
    (prog1
        (setf (getf *joint-trajectory-action-clients* left-or-right)
              (actionlib:make-action-client
               action-server-name
               "pr2_controllers_msgs/JointTrajectoryAction"))
      (loop until (actionlib:wait-for-server
                   (getf *joint-trajectory-action-clients* left-or-right)
                   5.0)
            do (roslisp:ros-info (joint-trajectory-action)
                                 "Waiting for ~a arm action server."
                                 left-or-right))
      (roslisp:ros-info (joint-trajectory-action)
                        "~a arm action client created."
                        left-or-right))))

(defun destroy-joint-trajectory-action-clients ()
  (setf *joint-trajectory-action-clients* '(:left nil :right nil)))

(roslisp-utilities:register-ros-cleanup-function destroy-joint-trajectory-action-clients)


(defun get-joint-trajectory-action-client (left-or-right)
  (or (getf *joint-trajectory-action-clients* left-or-right)
      (init-joint-trajectory-action-client left-or-right)))

(defun make-joint-trajectory-action-goal (left-or-right positions-vectors-.-times-list)
  "`positions-vectors-.-times-list' is e.g. '((#(0 0 0 0 0 0 0) . 0.0)
                                              (#(1 1 1 1 1 1 1) . 1.0))."
  (actionlib:make-action-goal
      (get-joint-trajectory-action-client left-or-right)
    (joint_names trajectory)
    (apply #'vector
           (mapcar (alexandria:curry
                    #'concatenate 'string (ecase left-or-right
                                            (:left "l")
                                            (:right "r")))
                   (list "_shoulder_pan_joint"
                         "_shoulder_lift_joint"
                         "_upper_arm_roll_joint"
                         "_elbow_flex_joint"
                         "_forearm_roll_joint"
                         "_wrist_flex_joint"
                         "_wrist_roll_joint")))

    (points trajectory)
    (apply #'vector
           (mapcar (lambda (positions-vector-.-time)
                     (roslisp:make-message
                      "trajectory_msgs/JointTrajectoryPoint"
                      positions (car positions-vector-.-time)
                      velocities (make-array 7 :initial-element 0)
                      time_from_start (cdr positions-vector-.-time)))
                   positions-vectors-.-times-list))))

(defun make-joint-trajectory-timestamps (position-vector-list)
  (maplist (let ((time-from-start 0.25))
             (lambda (position-vector-sublist)
               (let ((position-vector-1 (first position-vector-sublist))
                     (position-vector-2 (second position-vector-sublist)))
                 (prog1
                     time-from-start
                   (when (and position-vector-1 position-vector-2)
                    (let ((max-joint-move
                            (apply #'max
                                   (map 'list (lambda (x y)
                                                (abs (- x y)))
                                        position-vector-1 position-vector-2))))
                      (setf time-from-start
                            (+ time-from-start
                               (/ max-joint-move *max-joint-velocity*)))))))))
           position-vector-list))

(defun call-joint-trajectory-action (left-or-right position-vector-list)
  (multiple-value-bind (result status)
      (cpl:with-failure-handling
          ((simple-error (e)
             (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
             (init-joint-trajectory-action-client left-or-right)
             (cpl:retry)))
        (let ((actionlib:*action-server-timeout* 10.0)
              (action-client (get-joint-trajectory-action-client left-or-right)))
          (actionlib:call-goal
           action-client
           (make-joint-trajectory-action-goal
            left-or-right
            (mapcar #'cons
                    position-vector-list
                    (make-joint-trajectory-timestamps position-vector-list)))
           :timeout 10.0
           :feedback-cb #'print)))
    (roslisp:ros-info (joint-trajectory-action) "~a arm action finished." left-or-right)
    (values result status)))

(defun call-joint-angle-action (left-or-right joint-angle-list)
  (multiple-value-bind (result status)
      (cpl:with-failure-handling
          ((simple-error (e)
             (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
             (init-joint-trajectory-action-client left-or-right)
             (cpl:retry)))
        (let ((actionlib:*action-server-timeout* 10.0)
              (action-client (get-joint-trajectory-action-client left-or-right)))
          (actionlib:call-goal
           action-client
           (actionlib:make-action-goal
               action-client
             (:joint_names :trajectory)
             (apply #'vector
                    (mapcar (alexandria:curry
                             #'concatenate 'string (ecase left-or-right
                                                     (:left "l")
                                                     (:right "r")))
                            (list "_shoulder_pan_joint"
                                  "_shoulder_lift_joint"
                                  "_upper_arm_roll_joint"
                                  "_elbow_flex_joint"
                                  "_forearm_roll_joint"
                                  "_wrist_flex_joint"
                                  "_wrist_roll_joint")))

             (:points :trajectory)
             (vector (roslisp:make-message
                      "trajectory_msgs/JointTrajectoryPoint"
                      :positions (apply #'vector joint-angle-list)
                      :velocities (make-array 7 :initial-element 0)
                      :time_from_start 0.25)))
           :timeout 10.0
           :feedback-cb #'print)))
    (roslisp:ros-info (joint-trajectory-action) "~a arm action finished." left-or-right)
    (values result status)))

(defun call-joint-trajectory-action-example ()
  (call-joint-trajectory-action
   :left
   '(#(0 0 0 0 0 0 0)
     #(-0.3 0.2 -0.1 -1.2 1.5 -0.3 0.5)
     #(0 0 0 -1.2 1.5 -0.3 0.5))))

(defun call-joint-angle-action-example ()
  (call-joint-angle-action :right '(0 0 0 0 0 0 0)))
