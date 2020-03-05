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

(in-package :giskard)

(defun make-giskard-neck-joint-action-goal (joint-state)
  (declare (type list joint-state))
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
                                          :avoid_all_collisions)))))))

(defun get-neck-joint-names-and-positions-list (&optional joint-states)
  (if joint-states
      (list (mapcar #'first joint-states)
            (mapcar #'second joint-states))
      (let ((joint-names
              (cut:var-value '?joints
                             (cut:lazy-car
                              (prolog:prolog
                               `(and (rob-int:robot ?robot)
                                     (rob-int:robot-neck-joints ?robot ?joints)))))))
        (list joint-names
              (joints:joint-positions joint-names)))))

(defun ensure-giskard-neck-joint-input-parameters (neck-goal)
  (if (and (listp neck-goal) (= (length neck-goal) 6))
      (get-neck-joint-names-and-positions-list neck-goal)
      (and (roslisp:ros-warn (low-level giskard)
                             "Joint goal ~a was not a list of 6. Ignoring."
                             neck-goal)
           (get-neck-joint-names-and-positions-list))))

(defun ensure-giskard-neck-joint-goal-reached (status
                                          goal-configuration
                                          convergence-delta-joint)
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Giskard action preempted.")
    (return-from ensure-giskard-neck-joint-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (pr2-ll giskard-joint) "Giskard action timed out."))
  
  (when goal-configuration
    (let ((configuration (second (get-neck-joint-names-and-positions-list))))
      (unless (cram-tf:values-converged (joints:normalize-joint-angles
                                         configuration)
                                        (joints:normalize-joint-angles
                                         (mapcar #'second goal-configuration))
                                        convergence-delta-joint)
        (cpl:fail 'common-fail:manipulation-goal-not-reached
                  :description (format nil "Giskard did not converge to goal:~%~
                                                   ~a (~a)~%should have been at~%~a~%~
                                                   with delta-joint of ~a."
                                       "neck"
                                       (joints:normalize-joint-angles
                                        configuration)
                                       (joints:normalize-joint-angles
                                        (mapcar #'second goal-configuration))
                                       convergence-delta-joint))))))

(defun call-giskard-neck-action (&key goal-configuration action-timeout
                                   (convergence-delta-joint *giskard-convergence-delta-joint*))
  (declare (type list goal-configuration)
           (type (or null number) convergence-delta-joint))
  (let ((joint-state (ensure-giskard-neck-joint-input-parameters goal-configuration)))
    (multiple-value-bind (result status)
        (actionlib-client:call-simple-action-client
         'giskard-action
         :action-goal (make-giskard-neck-joint-action-goal joint-state)
         :action-timeout action-timeout)
      (ensure-giskard-neck-joint-goal-reached status goal-configuration
                                              convergence-delta-joint)
      (values result status)
      ;; return the joint state, which is our observation
      (joints:full-joint-states-as-hash-table))))


