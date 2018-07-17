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

(in-package :boxy-proj)

(defun make-giskard-action-client ()
  (actionlib-client:make-simple-action-client
   'giskard-action
   "qp_controller/command" "giskard_msgs/MoveAction"
   60))

(roslisp-utilities:register-ros-init-function make-giskard-action-client)

(defparameter *giskard-convergence-delta-joint* 0.17 "in radiants, about 10 degrees")

(defun make-giskard-joint-action-goal (joint-state-left joint-state-right)
  (declare (type list joint-state-left joint-state-right))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal :plan_and_execute)
   :cmd_seq (vector (roslisp:make-message
                     'giskard_msgs-msg:movecmd
                     :controllers (vector (roslisp:make-message
                                           'giskard_msgs-msg:controller
                                           :type (roslisp:symbol-code
                                                  'giskard_msgs-msg:controller
                                                  :joint)
                                           :p_gain 10
                                           :weight 1
                                           :max_speed (cma:degrees->radians 30)
                                           :goal_state (roslisp:make-message
                                                        'sensor_msgs-msg:jointstate
                                                        :name (apply #'vector
                                                                     (first
                                                                      joint-state-left))
                                                        :position (apply #'vector
                                                                         (second
                                                                          joint-state-left))))
                                          (roslisp:make-message
                                           'giskard_msgs-msg:controller
                                           :type (roslisp:symbol-code
                                                  'giskard_msgs-msg:controller
                                                  :joint)
                                           :p_gain 10
                                           :weight 1
                                           :max_speed (cma:degrees->radians 30)
                                           :goal_state (roslisp:make-message
                                                        'sensor_msgs-msg:jointstate
                                                        :name (apply #'vector
                                                                     (first
                                                                      joint-state-right))
                                                        :position (apply #'vector
                                                                         (second
                                                                          joint-state-right)))))
                     :collisions (vector (roslisp:make-message
                                          'giskard_msgs-msg:collisionentry
                                          :type (roslisp:symbol-code
                                                 'giskard_msgs-msg:collisionentry
                                                 :avoid_all_collisions)
                                          :min_dist 0.05))))))

;; (defun ensure-giskard-joint-input-parameters (left-goal right-goal)
;;   (flet ((ensure-giskard-joint-goal (goal arm)
;;            (if (and (listp goal) (= (length goal) 7))
;;                goal
;;                (and (roslisp:ros-warn (low-level giskard)
;;                                       "Joint goal ~a was not a list of 7. Ignoring."
;;                                       goal)
;;                     (get-arm-joint-states arm)))))
;;    (values (ensure-giskard-joint-goal left-goal :left)
;;            (ensure-giskard-joint-goal right-goal :right))))

(defun move-arms-giskard-joint (&key goal-configuration-left goal-configuration-right
                                  action-timeout)
  (declare (type list goal-configuration-left goal-configuration-right)
           (type (or null number) action-timeout))
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'giskard-action
       :action-goal (make-giskard-joint-action-goal
                     goal-configuration-left goal-configuration-right)
       :action-timeout action-timeout)
    (values result status)))



(defparameter *giskard-convergence-delta-xy* 0.005 "in meters")
(defparameter *giskard-convergence-delta-theta* 0.1 "in radiants, about 6 degrees")

(defun make-giskard-cartesian-action-goal (left-pose right-pose
                                           pose-base-frame left-tool-frame right-tool-frame
                                           collision-mode)
  (declare (type (or null cl-transforms-stamped:pose-stamped) left-pose right-pose)
           (type string pose-base-frame left-tool-frame right-tool-frame))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal :plan_and_execute)
   :cmd_seq (vector (roslisp:make-message
                     'giskard_msgs-msg:movecmd
                     :controllers
                     (map 'vector #'identity
                          (remove nil
                                  (list
                                   (when left-pose
                                     (roslisp:make-message
                                      'giskard_msgs-msg:controller
                                      :type (roslisp:symbol-code
                                             'giskard_msgs-msg:controller
                                             :translation_3d)
                                      :root_link pose-base-frame
                                      :tip_link left-tool-frame
                                      :p_gain 3
                                      :weight 1
                                      :max_speed 0.3
                                      :goal_pose (cl-transforms-stamped:to-msg left-pose)))
                                   (when left-pose
                                     (roslisp:make-message
                                      'giskard_msgs-msg:controller
                                      :type (roslisp:symbol-code
                                             'giskard_msgs-msg:controller
                                             :rotation_3d)
                                      :root_link pose-base-frame
                                      :tip_link left-tool-frame
                                      :p_gain 3
                                      :weight 1
                                      :max_speed (cma:degrees->radians 30)
                                      :goal_pose (cl-transforms-stamped:to-msg left-pose)))
                                   (when right-pose
                                     (roslisp:make-message
                                      'giskard_msgs-msg:controller
                                      :type (roslisp:symbol-code
                                             'giskard_msgs-msg:controller
                                             :translation_3d)
                                      :root_link pose-base-frame
                                      :tip_link right-tool-frame
                                      :p_gain 3
                                      :weight 1
                                      :max_speed 0.3
                                      :goal_pose (cl-transforms-stamped:to-msg right-pose)))
                                   (when right-pose
                                     (roslisp:make-message
                                      'giskard_msgs-msg:controller
                                      :type (roslisp:symbol-code
                                             'giskard_msgs-msg:controller
                                             :rotation_3d)
                                      :root_link pose-base-frame
                                      :tip_link right-tool-frame
                                      :p_gain 3
                                      :weight 1
                                      :max_speed (cma:degrees->radians 30)
                                      :goal_pose (cl-transforms-stamped:to-msg right-pose))))))
                     :collisions
                     (case collision-mode
                       (:allow-all
                        (vector (roslisp:make-message
                                 'giskard_msgs-msg:collisionentry
                                 :type (roslisp:symbol-code
                                        'giskard_msgs-msg:collisionentry
                                        :allow_all_collisions))))
                       (:avoid-all
                        (vector (roslisp:make-message
                                 'giskard_msgs-msg:collisionentry
                                 :type (roslisp:symbol-code
                                        'giskard_msgs-msg:collisionentry
                                        :avoid_all_collisions)
                                 :min_dist 0.05)))
                       (t
                        (vector (roslisp:make-message
                                 'giskard_msgs-msg:collisionentry
                                 :type (roslisp:symbol-code
                                        'giskard_msgs-msg:collisionentry
                                        :avoid_all_collisions)
                                 :min_dist 0.05))))))))

(defun ensure-giskard-cartesian-input-parameters (frame left-pose right-pose)
  (values (when left-pose
            (cram-tf:ensure-pose-in-frame left-pose frame))
          (when right-pose
            (cram-tf:ensure-pose-in-frame right-pose frame))))

(defun ensure-giskard-cartesian-goal-reached (result status goal-position-left goal-position-right
                                              goal-frame-left goal-frame-right
                                              convergence-delta-xy convergence-delta-theta)
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Giskard action preempted.")
    (return-from ensure-giskard-cartesian-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (pr2-ll giskard-cart) "Giskard action timed out."))
  (when (eql status :aborted)
    (roslisp:ros-warn (pr2-ll giskard-cart) "Giskard action aborted! With result ~a" result)
    ;; (cpl:fail 'common-fail:manipulation-goal-not-reached
    ;;           :description "Giskard did not converge to goal because of collision")
    )
  (when goal-position-left
    (unless (cram-tf:tf-frame-converged goal-frame-left goal-position-left
                                        convergence-delta-xy convergence-delta-theta)
      (cpl:fail 'common-fail:manipulation-goal-not-reached
                :description (format nil "Giskard did not converge to goal:
~a should have been at ~a with delta-xy of ~a and delta-angle of ~a."
                                     goal-frame-left goal-position-left
                                     convergence-delta-xy convergence-delta-theta))))
  (when goal-position-right
    (unless (cram-tf:tf-frame-converged goal-frame-right goal-position-right
                                        convergence-delta-xy convergence-delta-theta)
      (cpl:fail 'common-fail:manipulation-goal-not-reached
                :description (format nil "Giskard did not converge to goal:
~a should have been at ~a with delta-xy of ~a and delta-angle of ~a."
                                     goal-frame-right goal-position-right
                                     convergence-delta-xy convergence-delta-theta)))))

(defun move-arms-giskard-cartesian (&key
                                      goal-pose-left goal-pose-right action-timeout
                                      collision-mode
                                      (pose-base-frame cram-tf:*robot-base-frame*)
                                      (left-tool-frame cram-tf:*robot-left-tool-frame*)
                                      (right-tool-frame cram-tf:*robot-right-tool-frame*)
                                      (convergence-delta-xy *giskard-convergence-delta-xy*)
                                      (convergence-delta-theta *giskard-convergence-delta-theta*))
  (declare (type (or null cl-transforms-stamped:pose-stamped) goal-pose-left goal-pose-right)
           (type (or null number) action-timeout convergence-delta-xy convergence-delta-theta)
           (type (or null string) pose-base-frame left-tool-frame right-tool-frame))
  (if (or goal-pose-left goal-pose-right)
      (multiple-value-bind (goal-pose-left goal-pose-right)
          (ensure-giskard-cartesian-input-parameters pose-base-frame goal-pose-left goal-pose-right)
        (cram-tf:visualize-marker (list goal-pose-left goal-pose-right) :r-g-b-list '(1 0 1))
        (multiple-value-bind (result status)
            (let ((goal (make-giskard-cartesian-action-goal
                         goal-pose-left goal-pose-right
                         pose-base-frame left-tool-frame right-tool-frame
                         collision-mode)))
              (actionlib-client:call-simple-action-client
               'giskard-action
               :action-goal goal
               :action-timeout action-timeout))
          (ensure-giskard-cartesian-goal-reached result status goal-pose-left goal-pose-right
                                                 left-tool-frame right-tool-frame
                                                 convergence-delta-xy convergence-delta-theta)
          (values result status)))
      (roslisp:ros-info (pr2-ll giskard-cart) "Got an empty goal...")))



#+old-giskard
(
 (defun make-giskard-action-client ()
   (actionlib-client:make-simple-action-client
    'giskard-action
    "controller_action_server/move" "giskard_msgs/WholeBodyAction"
    60))

 (roslisp-utilities:register-ros-init-function make-giskard-action-client)

 (defparameter *giskard-convergence-delta-joint* 0.17 "in radiants, about 10 degrees")

 (defun make-giskard-joint-action-goal (left-configuration right-configuration
                                        &optional convergence-delta-joint)
   (declare (type list left-configuration right-configuration)
            (ignore convergence-delta-joint))
   (roslisp:make-message
    'giskard_msgs-msg:WholeBodyGoal
    (:type :command)
    (roslisp:symbol-code 'giskard_msgs-msg:wholebodycommand :standard_controller)
    (:type :left_ee :command)
    (roslisp:symbol-code 'giskard_msgs-msg:armcommand :joint_goal)
    (:goal_configuration :left_ee :command)
    (apply #'vector left-configuration)
    (:type :right_ee :command)
    (roslisp:symbol-code 'giskard_msgs-msg:armcommand :joint_goal)
    (:goal_configuration :right_ee :command)
    (apply #'vector right-configuration)))

 ;; (defun ensure-giskard-joint-input-parameters (left-goal right-goal)
 ;;   (flet ((ensure-giskard-joint-goal (goal arm)
 ;;            (if (and (listp goal) (= (length goal) 7))
 ;;                goal
 ;;                (and (roslisp:ros-warn (low-level giskard)
 ;;                                       "Joint goal ~a was not a list of 7. Ignoring."
 ;;                                       goal)
 ;;                     (get-arm-joint-states arm)))))
 ;;    (values (ensure-giskard-joint-goal left-goal :left)
 ;;            (ensure-giskard-joint-goal right-goal :right))))

 (defun move-arms-giskard-joint (&key goal-configuration-left goal-configuration-right
                                   action-timeout)
   (declare (type list goal-configuration-left goal-configuration-right)
            (type (or null number) action-timeout))
   (multiple-value-bind (result status)
       (actionlib-client:call-simple-action-client
        'giskard-action
        :action-goal (make-giskard-joint-action-goal
                      goal-configuration-left goal-configuration-right)
        :action-timeout action-timeout)
     (values result status)))



 (defparameter *giskard-convergence-delta-xy* 0.005 "in meters")
 (defparameter *giskard-convergence-delta-theta* 0.1 "in radiants, about 6 degrees")

 (defun make-giskard-cartesian-action-goal (left-pose right-pose
                                            &optional convergence-delta-xy convergence-delta-theta)
   (declare (ignore convergence-delta-xy convergence-delta-theta)
            (type (or cl-transforms:pose cl-transforms-stamped:pose-stamped)
                  left-pose right-pose))
   (roslisp:make-message
    'giskard_msgs-msg:WholeBodyGoal
    (:type :command)
    (roslisp:symbol-code 'giskard_msgs-msg:wholebodycommand :standard_controller)
    (:type :left_ee :command)
    (roslisp:symbol-code 'giskard_msgs-msg:armcommand :cartesian_goal)
    (:goal_pose :left_ee :command)
    (cl-transforms-stamped:to-msg left-pose)
    ;; (:type :right_ee :command)
    ;; (roslisp:symbol-code 'giskard_msgs-msg:armcommand :joint_goal)
    ;; (:goal_configuration :right_ee :command)
    ;; ;; sending right arm goal as a joint state because our right arm is broken at the moment
    ;; (apply #'vector (get-arm-joint-states :right))
    ))

 (defun ensure-giskard-cartesian-input-parameters (frame left-pose right-pose)
   (values (cram-tf:ensure-pose-in-frame
            (or left-pose
                (cl-transforms-stamped:pose->pose-stamped
                 cram-tf:*robot-left-tool-frame*
                 0.0
                 (cl-transforms:make-identity-pose)))
            frame)
           (cram-tf:ensure-pose-in-frame
            (or right-pose
                (cl-transforms-stamped:pose->pose-stamped
                 cram-tf:*robot-right-tool-frame*
                 0.0
                 (cl-transforms:make-identity-pose)))
            frame)))

 (defun ensure-giskard-cartesian-goal-reached (status goal-position-left goal-position-right
                                               goal-frame-left goal-frame-right
                                               convergence-delta-xy convergence-delta-theta)
   (when (eql status :timeout)
     (cpl:fail 'common-fail:actionlib-action-timed-out :description "Giskard action timed out"))
   (when (eql status :preempted)
     (roslisp:ros-warn (low-level giskard) "Giskard action preempted.")
     (return-from ensure-giskard-cartesian-goal-reached))
   (when goal-position-left
     (unless (cram-tf:tf-frame-converged goal-frame-left goal-position-left
                                         convergence-delta-xy convergence-delta-theta)
       (cpl:fail 'common-fail:manipulation-goal-not-reached
                 :description (format nil "Giskard did not converge to goal: ~
                                          ~a should have been at ~a with delta-xy of ~a ~
                                          and delta-angle of ~a."
                                      goal-frame-left goal-position-left
                                      convergence-delta-xy convergence-delta-theta)))))

 (defun move-arms-giskard-cartesian (&key
                                       goal-pose-left goal-pose-right action-timeout
                                       (pose-base-frame cram-tf:*robot-base-frame*)
                                       (left-tool-frame cram-tf:*robot-left-tool-frame*)
                                       (right-tool-frame cram-tf:*robot-right-tool-frame*)
                                       (convergence-delta-xy *giskard-convergence-delta-xy*)
                                       (convergence-delta-theta *giskard-convergence-delta-theta*))
   (declare (type (or null cl-transforms-stamped:pose-stamped) goal-pose-left goal-pose-right)
            (type (or null string) pose-base-frame left-tool-frame right-tool-frame)
            (type (or null number) action-timeout convergence-delta-xy convergence-delta-theta))
   (multiple-value-bind (goal-pose-left goal-pose-right)
       (ensure-giskard-cartesian-input-parameters pose-base-frame goal-pose-left goal-pose-right)
     (multiple-value-bind (result status)
         (actionlib-client:call-simple-action-client
          'giskard-action
          :action-goal (make-giskard-cartesian-action-goal
                        goal-pose-left goal-pose-right
                        convergence-delta-xy convergence-delta-theta)
          :action-timeout action-timeout)
       (ensure-giskard-cartesian-goal-reached status goal-pose-left goal-pose-right
                                              left-tool-frame right-tool-frame
                                              convergence-delta-xy convergence-delta-theta)
       (values result status))))

 )
