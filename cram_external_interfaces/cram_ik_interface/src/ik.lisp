;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Christopher Pollok <cpollok@cs.uni-bremen.de>
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

(in-package :ik)

(defparameter *ik-service-name* "kdl_ik_service/get_ik")

(defun call-ik-service (cartesian-pose base-link end-effector-link seed-state-msg)
  (declare (type cl-transforms-stamped:pose-stamped cartesian-pose)
           (type string base-link end-effector-link)
           (type (or null sensor_msgs-msg:jointstate) seed-state-msg))
  (handler-case
      (roslisp:with-fields ((response-error-code (val error_code))
                            (joint-state (joint_state solution)))
          (progn
            (roslisp:wait-for-service *ik-service-name* 10.0)
            (roslisp:call-service
             *ik-service-name*
             'moveit_msgs-srv:getpositionik
             (roslisp:make-request
              'moveit_msgs-srv:getpositionik
              (:ik_link_name :ik_request) end-effector-link
              (:pose_stamped :ik_request) (cl-transforms-stamped:to-msg
                                           (cram-tf:ensure-pose-in-frame
                                            cartesian-pose base-link :use-zero-time t))
              (:joint_state :robot_state :ik_request) seed-state-msg
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
                                          response-error-code))))))
    ;; PR2's IK kinematics solver sends garbage sometimes
    (simple-error (e)
      (declare (ignore e))
      nil)))

(defun call-ik-service-with-resampling (cartesian-pose
                                        base-link end-effector-link
                                        seed-state-msg
                                        resampling-step resampling-axis
                                        current-value lower-limit upper-limit)
  (labels ((call-ik-service-with-resampling-inner (cartesian-pose
                                                   &key test-value current-value)
             (let ((ik-solution-msg
                     (call-ik-service cartesian-pose base-link end-effector-link seed-state-msg)))
               (if ik-solution-msg
                   (values ik-solution-msg
                           (or test-value current-value))
                   (when (or (not test-value) (> test-value lower-limit))
                     ;; When we have no ik solution and have a valid test value to try,
                     ;; use it to resample.
                     (let* ((next-test-value
                              (if test-value
                                  (max lower-limit (- test-value resampling-step))
                                  upper-limit))
                            (offset
                              (if test-value
                                  (- test-value current-value)
                                  0))
                            (next-offset
                              (- next-test-value current-value))
                            (pseudo-pose
                              (cram-tf:translate-pose
                               cartesian-pose
                               (ecase resampling-axis
                                 (:x :x-offset)
                                 (:y :y-offset)
                                 (:z :z-offset))
                               (- offset next-offset))))
                       (call-ik-service-with-resampling-inner
                        pseudo-pose
                        :test-value next-test-value
                        :current-value current-value)))))))

    (let ((old-debug-lvl (roslisp:debug-level nil)))
      (unwind-protect
           (progn
             (roslisp:set-debug-level nil 9)
             (call-ik-service-with-resampling-inner
              cartesian-pose
              :test-value nil
              :current-value current-value))
        (roslisp:set-debug-level nil old-debug-lvl)))))


(defun call-ik-service-with-resampling-and-collision-check (cartesian-pose
                                                            base-link end-effector-link
                                                            seed-state-msg
                                                            resampling-step resampling-axis
                                                            current-value lower-limit upper-limit
                                                            &key collision-check-function
                                                            collision-check-arguments)
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))
    (labels ((call-ik-service-inner (cartesian-pose
                                     &key test-value current-value)
               (let ((ik-solution-msg
                       (call-ik-service
                        cartesian-pose base-link end-effector-link seed-state-msg)))
                 (if ik-solution-msg
                     (cpl:with-failure-handling
                         ((common-fail:manipulation-goal-not-reached (e)
                            (declare (ignore e))
                            (btr::restore-world-state world-state world)
                            (call-ik-service-inner-with-next-solution cartesian-pose
                                                                      :test-value test-value
                                                                      :current-value current-value)))
                       (apply collision-check-function collision-check-arguments)
                       (values ik-solution-msg
                               (or test-value current-value)))
                     (when (or (not test-value) (> test-value lower-limit))
                       ;; When we have no ik solution and have a valid test value to try,
                       ;; use it to resample.
                       (call-ik-service-inner-with-next-solution cartesian-pose
                                                                 :test-value test-value
                                                                 :current-value current-value)))))
             (call-ik-service-inner-with-next-solution (cartesian-pose
                                                        &key test-value current-value)
               (let* ((next-test-value
                        (if test-value
                            (max lower-limit (- test-value resampling-step))
                            upper-limit))
                      (offset
                        (if test-value
                            (- test-value current-value)
                            0))
                      (next-offset
                        (- next-test-value current-value))
                      (pseudo-pose
                        (cram-tf:translate-pose
                         cartesian-pose
                         (ecase resampling-axis
                           (:x :x-offset)
                           (:y :y-offset)
                           (:z :z-offset))
                         (- offset next-offset))))
                 (call-ik-service-inner
                  pseudo-pose
                  :test-value next-test-value
                  :current-value current-value))))
      
      (let ((old-debug-lvl (roslisp:debug-level nil)))
        (unwind-protect
             (progn
               (roslisp:set-debug-level nil 9)
               (call-ik-service-inner
                cartesian-pose
                :test-value nil
                :current-value current-value))
          (progn
            (btr::restore-world-state world-state world)
            (roslisp:set-debug-level nil old-debug-lvl)))))))
