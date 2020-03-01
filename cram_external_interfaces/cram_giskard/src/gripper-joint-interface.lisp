;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2020, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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

;; THOSE SHOULD GO TO BOXY DESCRIPTION WITH A WRAPPING PREDICATE
(defparameter *gripper-minimal-position* 0.007 "in meters")
(defparameter *gripper-maximal-position* 0.108 "in meters")
(defparameter *giskard-gripper-convergence-delta-joint* 0.005 "converge at 5mm accuracy")

(defun make-giskard-gripper-joint-action-goal (joint-state effort)
  (declare (type list joint-state)
           (ignore effort)) ;; include in constraint
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal :plan_and_execute)
   :cmd_seq (vector
             (roslisp:make-message
              'giskard_msgs-msg:movecmd
              :joint_constraints (vector (roslisp:make-message
                                          'giskard_msgs-msg:jointconstraint
                                          :type (roslisp:symbol-code
                                                 'giskard_msgs-msg:jointconstraint
                                                 :joint)
                                          :goal_state (roslisp:make-message
                                                       'sensor_msgs-msg:jointstate
                                                       :name (apply #'vector
                                                                    (first
                                                                     joint-state))
                                                       :position (apply #'vector
                                                                        (second
                                                                         joint-state)))))
              :collisions (vector (roslisp:make-message
                                   'giskard_msgs-msg:collisionentry
                                   :type (roslisp:symbol-code
                                          'giskard_msgs-msg:collisionentry
                                          :allow_all_collisions)))))))

(defun get-gripper-joint-and-position-list (left-or-right &optional joint-state)
  "Creates joint-states list like this
'((gripper-joint-name) (gripper-joint-position))"
  (let ((joint-name
              (cut:var-value '?joint
                             (cut:lazy-car
                              (prolog:prolog
                               `(rob-int:gripper-joint
                                 ,(rob-int:current-robot-symbol)
                                 ,left-or-right ?joint))))))
    (if joint-state
        `((,joint-name)
          (,joint-state))
        `((,joint-name) ,(joints:joint-positions `(,joint-name))))))
      

(defun ensure-gripper-input-parameters (action-type-or-position left-or-right effort)
  (declare (type (or number keyword) action-type-or-position)
           (type keyword left-or-right)
           (type (or null number) effort))
  (let* ((position
          (etypecase action-type-or-position
            (number
             (cond
               ((< action-type-or-position *gripper-minimal-position*)
                (roslisp:ros-warn (gripper-action)
                                  "POSITION (~a) cannot be < ~a. Clipping."
                                  action-type-or-position *gripper-minimal-position*)
                *gripper-minimal-position*)
               ((> action-type-or-position *gripper-maximal-position*)
                (roslisp:ros-warn (gripper-action)
                                  "POSITION (~a) shouldn't be > ~a. Clipping."
                                  action-type-or-position *gripper-maximal-position*)
                *gripper-maximal-position*)
               (t
                action-type-or-position)))
            (keyword
             (ecase action-type-or-position
               (:open *gripper-maximal-position*)
               (:close *gripper-minimal-position*)
               (:grip *gripper-minimal-position*)))))
        (effort
          (or effort
              (etypecase action-type-or-position
                (number 30.0)
                (keyword (ecase action-type-or-position
                           (:open 30.0)
                           (:close 30.0)
                           (:grip 15.0))))))
         (joint-state (get-gripper-joint-and-position-list left-or-right position)))
    (values joint-state effort)))

(defun ensure-giskard-gripper-joint-goal-reached (status
                                                  joint-goal
                                                  left-or-right
                                                  convergence-delta-joint)
  (when (eql status :preempted)
    (roslisp:ros-warn (giskard-ll giskard-gripper-joint) "Giskard action preempted.")
    (return-from ensure-giskard-gripper-joint-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (giskard-ll giskard-gripper-joint) "Giskard action timed out."))
  (when (and joint-goal left-or-right)
    (let ((current-state (second (get-gripper-joint-and-position-list left-or-right)))
          (goal-state (second joint-goal)))
      (unless (cram-tf:values-converged current-state
                                        goal-state
                                        convergence-delta-joint)
        (cpl:fail 'common-fail:manipulation-goal-not-reached
                  :description (format nil "Giskard did not converge to goal:~%~
                                            ~a (~a)~%should have been at~%~a~%~
                                            with delta-joint of ~a."
                                       left-or-right
                                       current-state
                                       goal-state
                                       convergence-delta-joint))))))

(defun call-giskard-gripper-action (&key action-type-or-position left-or-right effort
                                      action-timeout
                                      (convergence-delta-joint *giskard-gripper-convergence-delta-joint*))
  (declare (type keyword left-or-right)
           ;; (type (or keyword list) left-or-right)
           (type (or null number) effort)
           (type (or number keyword) action-type-or-position))
  "`goal-position' is in meters."
  
  (multiple-value-bind (joint-goal effort)
      (ensure-gripper-input-parameters action-type-or-position left-or-right effort)
    
    (multiple-value-bind (result status)
        (actionlib-client:call-simple-action-client
         'giskard-action
         :action-goal (make-giskard-gripper-joint-action-goal joint-goal effort)
         :action-timeout action-timeout)

      (cpl:sleep 1.0) ;; FIXME: wait for action result before ensuring
      (ensure-giskard-gripper-joint-goal-reached status joint-goal left-or-right
                                                 convergence-delta-joint)
      (values result status)
      ;; return the joint state, which is our observation
      (joints:full-joint-states-as-hash-table))))
