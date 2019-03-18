;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :hsrb-proj)

(defparameter *ik-service-name* "hsrb_arm_kinematics/get_ik")

(defun call-ik-service (left-or-right cartesian-pose &optional seed-state)
  (declare (type keyword left-or-right)
           (type cl-transforms-stamped:pose-stamped cartesian-pose)
           (ignore seed-state))
  (let* ((robot-info-bindings
           (cut:lazy-car
            (prolog:prolog
             `(and (cram-robot-interfaces:robot ?robot)
                   (cram-robot-interfaces:arm-joints ?robot ,left-or-right ?joints)
                   (cram-robot-interfaces:end-effector-link ?robot ,left-or-right ?ee-link)
                   (cram-robot-interfaces:robot-torso-link-joint ?robot ?torso-link ?_)))))
         (ee-link
           (cut:var-value '?ee-link robot-info-bindings))
         (torso-link
           (cut:var-value '?torso-link robot-info-bindings))
         (joint-names
           (cut:var-value '?joints robot-info-bindings))
         (joint-state-msg
           (btr::make-robot-joint-state-msg
            (btr:get-robot-object)
            :joint-names joint-names)))
    
    (roslisp:with-fields ((response-error-code (val error_code))
                          (joint-state (joint_state solution)))
        (progn
          (roslisp:wait-for-service *ik-service-name* 10.0)
          (roslisp:call-service
           *ik-service-name*
           'moveit_msgs-srv:getpositionik
           (roslisp:make-request
            'moveit_msgs-srv:getpositionik
            (:ik_link_name :ik_request) ee-link
            (:pose_stamped :ik_request) (cl-transforms-stamped:to-msg
                                         (cram-tf:ensure-pose-in-frame
                                          cartesian-pose torso-link :use-zero-time t))
            (:joint_state :robot_state :ik_request) joint-state-msg
            (:timeout :ik_request) 1.0)))
      (cond ((eql response-error-code
                  (roslisp-msg-protocol:symbol-code
                   'moveit_msgs-msg:moveiterrorcodes
                   :success))
             joint-state)
            ((eql response-error-code
                  (roslisp-msg-protocol:symbol-code
                   'moveit_msgs-msg:moveiterrorcodes
                   :no_ik_solution))
             nil)
            (T
             (error 'simple-error
                    :format-control "IK service failed: ~a"
                    :format-arguments (list
                                       (roslisp-msg-protocol:code-symbol
                                        'moveit_msgs-msg:moveiterrorcodes
                                        response-error-code))))))))

(defparameter *torso-step* 0.01)

(defun call-ik-service-with-torso-resampling (left-or-right cartesian-pose
                                              &key seed-state test-angle torso-angle
                                                torso-lower-limit torso-upper-limit)
  (labels ((call-ik-service-with-torso-resampling-inner (left-or-right cartesian-pose
                                                         &key seed-state test-angle
                                                           torso-angle
                                                           torso-lower-limit torso-upper-limit)
             (let ((ik-solution (call-ik-service left-or-right cartesian-pose seed-state)))
               (if (not ik-solution)
                   (when (or (not test-angle) (> test-angle torso-lower-limit))
                     ;; When we have no ik solution and have a valid torso angle to try,
                     ;; use it to resample.
                     (let* ((next-test-angle
                              (if test-angle
                                  (max torso-lower-limit (- test-angle *torso-step*))
                                  torso-upper-limit))
                            (torso-offset
                              (if test-angle
                                  (- test-angle torso-angle)
                                  0))
                            (next-torso-offset
                              (- next-test-angle torso-angle))
                            (pseudo-pose
                              (cram-tf:translate-pose cartesian-pose
                                                      :z-offset (- torso-offset
                                                                   next-torso-offset))))
                       (call-ik-service-with-torso-resampling-inner
                        left-or-right pseudo-pose
                        :seed-state seed-state
                        :test-angle next-test-angle
                        :torso-angle torso-angle
                        :torso-lower-limit torso-lower-limit
                        :torso-upper-limit torso-upper-limit)))
                   (values ik-solution (or test-angle torso-angle))))))

    (let ((old-debug-lvl (roslisp:debug-level NIL)))
      (unwind-protect
           (progn
             (roslisp:set-debug-level NIL 9)
             (call-ik-service-with-torso-resampling-inner
              left-or-right cartesian-pose
              :seed-state seed-state
              :test-angle test-angle
              :torso-angle torso-angle
              :torso-lower-limit torso-lower-limit
              :torso-upper-limit torso-upper-limit))
        (roslisp:set-debug-level NIL old-debug-lvl)))))



#+commentedoutfornow
(
 (defun set-robot-state-to-ik-result ()
   (btr:set-robot-state-from-joints
    (call-ik)
    (btr:get-robot-object)))

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
 )
