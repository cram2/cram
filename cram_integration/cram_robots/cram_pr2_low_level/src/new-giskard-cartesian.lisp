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

(defparameter *giskard-convergence-delta-xy* 0.005 "in meters")
(defparameter *giskard-convergence-delta-theta* 0.1 "in radiants, about 6 degrees")

(defun make-giskard-cartesian-action-goal (left-pose right-pose
                                           pose-base-frame left-tool-frame right-tool-frame
                                           collision-mode)
  (declare (type (or null cl-transforms-stamped:pose-stamped) left-pose right-pose)
           (type string pose-base-frame left-tool-frame right-tool-frame))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal ;; :plan_only
                              :plan_and_execute)
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
                       (:allow-hand
                        (apply #'vector
                               (roslisp:make-message
                                'giskard_msgs-msg:collisionentry
                                :type (roslisp:symbol-code
                                       'giskard_msgs-msg:collisionentry
                                       :avoid_all_collisions)
                                :min_dist 0.05)
                               (mapcar (lambda (robot-link)
                                         (roslisp:make-message
                                          'giskard_msgs-msg:collisionentry
                                          :type (roslisp:symbol-code
                                                 'giskard_msgs-msg:collisionentry
                                                 :allow_collision)
                                          :robot_link robot-link
                                          :body_b "kitchen"))
                                       (append
                                        (when left-pose
                                          (append
                                           (cram-pr2-description:get-hand-link-names :left)
                                           '("l_forearm_link"
                                             "l_wrist_flex_link"
                                             "l_wrist_roll_link")))
                                        (when right-pose
                                          (append
                                           (cram-pr2-description:get-hand-link-names :right)
                                           '("r_forearm_link"
                                             "r_wrist_flex_link"
                                             "r_wrist_roll_link")))))))
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

(defun ensure-giskard-cartesian-goal-reached (status goal-position-left goal-position-right
                                              goal-frame-left goal-frame-right
                                              convergence-delta-xy convergence-delta-theta)
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Giskard action preempted.")
    (return-from ensure-giskard-cartesian-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (pr2-ll giskard-cart) "Giskard action timed out."))
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

(defun call-giskard-cartesian-action (&key
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
  (when (or goal-pose-left goal-pose-right)
    (multiple-value-bind (goal-pose-left goal-pose-right)
        (ensure-giskard-cartesian-input-parameters pose-base-frame goal-pose-left goal-pose-right)
      (visualize-marker (list goal-pose-left goal-pose-right) :r-g-b-list '(1 0 1))
      (multiple-value-bind (result status)
          (actionlib-client:call-simple-action-client
           'giskard-action
           :action-goal (make-giskard-cartesian-action-goal
                         goal-pose-left goal-pose-right
                         pose-base-frame left-tool-frame right-tool-frame
                         collision-mode)
           :action-timeout action-timeout)
        (ensure-giskard-cartesian-goal-reached status goal-pose-left goal-pose-right
                                               left-tool-frame right-tool-frame
                                               convergence-delta-xy convergence-delta-theta)
        (values result status)))))

