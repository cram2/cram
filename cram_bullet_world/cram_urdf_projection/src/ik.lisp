;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :urdf-proj)

(defparameter *ik-service-name* '(:left "boxy_arm_kinematics/get_ik"
                                  :right "boxy_arm_kinematics/get_ik"))

;; (defparameter *ik-service-name* '(:left "pr2_left_arm_kinematics/get_ik"
;;                                   :right "pr2_right_arm_kinematics/get_ik"))

(defun call-ik-service (left-or-right cartesian-pose &optional seed-state)
  (declare (type keyword left-or-right)
           (type cl-transforms-stamped:pose-stamped cartesian-pose)
           (type (or null sensor_msgs-msg:jointstate) seed-state))
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
           (or seed-state
               (btr::make-robot-joint-state-msg
                (btr:get-robot-object)
                :joint-names joint-names))))
    (roslisp:with-fields ((response-error-code (val error_code))
                          (joint-state (joint_state solution)))
        (progn
          (roslisp:wait-for-service (getf *ik-service-name* left-or-right) 10.0)
          (roslisp:call-service
           (getf *ik-service-name* left-or-right)
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
             (let ((ik-solution-msg (call-ik-service left-or-right cartesian-pose seed-state)))
               (if ik-solution-msg
                   (let ((ik-solution-position
                           (map 'list #'identity
                                (roslisp:msg-slot-value
                                 ik-solution-msg 'sensor_msgs-msg:position)))
                         (resulting-torso-angle (or test-angle torso-angle)))
                     (values ik-solution-position resulting-torso-angle))
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
                        :torso-upper-limit torso-upper-limit)))))))

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


#+might-be-really-outdated-now
(defmethod cram-robot-interfaces:compute-iks (pose-stamped
                                              &key link-name arm robot-state seed-state
                                                (pose-stamped-frame
                                                 cram-tf:*robot-torso-frame*)
                                                (tcp-in-ee-pose
                                                 (cl-transforms:make-identity-pose)))
  (declare (ignore link-name robot-state))
  (let* ((tcp-pose (cl-transforms-stamped:transform-pose-stamped
                    cram-tf:*transformer*
                    :pose (cl-transforms-stamped:copy-pose-stamped
                           pose-stamped :stamp 0.0)
                    :target-frame pose-stamped-frame
                    :timeout cram-tf:*tf-default-timeout*))
         (goal-trans (cl-transforms:transform*
                      (cl-transforms:reference-transform tcp-pose)
                      (cl-transforms:transform-inv
                       (cl-transforms:reference-transform tcp-in-ee-pose))))
         (ee-pose (cl-transforms-stamped:make-pose-stamped
                   (cl-transforms-stamped:frame-id tcp-pose)
                   (cl-transforms-stamped:stamp tcp-pose)
                   (cl-transforms:translation goal-trans)
                   (cl-transforms:rotation goal-trans)))
         (solution
           (call-ik-service arm
                            ee-pose
                            seed-state)))
    (when solution
      (list solution))))
