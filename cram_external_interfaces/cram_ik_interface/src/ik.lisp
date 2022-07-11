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
(defparameter *float-comparison-precision* 0.001d0)

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
               nil)))
    ;; PR2's IK kinematics solver sends garbage sometimes
    (simple-error (e)
      (declare (ignore e))
      nil)))

(defun call-ik-service-with-resampling (cartesian-pose
                                        base-link end-effector-link
                                        seed-state-msg
                                        resampling-step resampling-axis
                                        current-value lower-limit upper-limit
                                        &optional solution-valid-p)
  (declare (type (or function null) solution-valid-p))
  "Calls the IK service to achieve the specified `cartesian-pose' by manipulating
the joints from `base-link' to `end-effector-link'.
`solution-valid-p' is a predicate that gets the joint state message as input
and returns T or NIL according to some validation criteria.
Mostly this is a collision check.
Returns two values: resulting joint state message and torso value.
If not valid solution was found, returns NIL."
  (labels ((call-ik-service-with-resampling-inner (cartesian-pose
                                                   &key test-value current-value)
             (let ((ik-solution-msg
                     (call-ik-service cartesian-pose base-link end-effector-link
                                      seed-state-msg)))
               (if (and ik-solution-msg
                        ;; if `solution-valid-p' is bound, call it
                        ;; otherwise assume that the solution is valid
                        (or (not solution-valid-p)
                            (funcall solution-valid-p ik-solution-msg)))
                   (values ik-solution-msg
                           (or test-value current-value))
                   (when (or (not test-value) (> test-value lower-limit))
                     ;; When we have no ik solution and have
                     ;; a valid test value to try, use it to resample.
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
                               resampling-axis (- offset next-offset))))
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


(defun get-joint-values-to-sample (lower-lim upper-lim sampling-step)
  (declare (type number lower-lim upper-lim sampling-step))
  "Returns a list of values such that they lie within [`lower-lim'; `upper-lim']
and the distance between the steps is `sampling-step'.
0.0 is always added to the list.
The values are sorted in ascending order based on their absolute magnitude.
E.g., if `lower-lim' = -4 and `upper-lim' = 4 and `sampling-step' = 1,
then return = (0.0 -1 1 -2 2 -3 3 -4 4)"
  (remove-duplicates
   (cons 0.0
         (sort
          (append (loop for x = lower-lim then (+ x sampling-step)
                        until (> x upper-lim)
                        collect x)
                  (list upper-lim))
          (lambda (x y)
            (< (abs x) (abs y)))))
   ;; remove duplicates in case current value is
   ;; exactly one of the sampling values
   :test (lambda (x y)
           (< (abs (- x y))
              *float-comparison-precision*))
   :from-end t))


(defmacro find-ik-for ((goal-pose base-link tip-link seed-state-message
                        &optional solution-valid-p)
                       &body body)
  "Method to find inverse kinematics for a given cartesian pose using resampling
Syntax:
 (ik::find-ik-for (goal-pose base-link tip-link seed-state-message)
         (ik::with-resampling (current-value resampling-axis
                               upper-limit lower-limit resampling-step)
             (ik::with-resampling current-value2 resampling-axis2 ...)
                            ....))
Resampling axis can only be :X, :Y or :Z"
  `(let ((offseted-goal-pose ,goal-pose)
         (base-link-evaled ,base-link)
         (tip-link-evaled ,tip-link)
         (seed-state-msg-evaled ,seed-state-message)
         (solution-valid-p-evaled ,solution-valid-p)
         (old-debug-level (roslisp:debug-level nil))
         new-joint-values)
     (macrolet
         ((with-resampling (&whole whole-form
                              (resampling-axis upper-limit lower-limit
                               resampling-step
                               &key (disable-resampling nil))
                            &body body)
            (let ((form-length (length whole-form)))
              ;; Formulating a list of joint values to sample
              `(let* ((original-goal-pose
                        offseted-goal-pose)
                      (sampling-values
                        (if ,disable-resampling
                            ;; stay at the default value if resampling is disalbled
                            (list 0.0d0)
                            (get-joint-values-to-sample
                             ,lower-limit ,upper-limit ,resampling-step)))
                      (result
                        (loop for value in sampling-values
                              do (setf offseted-goal-pose
                                       (cram-tf:translate-pose
                                        original-goal-pose
                                        ,resampling-axis
                                        (- value)))
                                 (if (assoc ,resampling-axis new-joint-values)
                                     (setf (cdr (assoc ,resampling-axis
                                                       new-joint-values))
                                           value)
                                     (setf new-joint-values
                                           (cons (cons ,resampling-axis value)
                                                 new-joint-values)))
                                 (multiple-value-bind
                                       (solution-msg joint-values)

                                     ;; Checking if the arguments contain
                                     ;; &body clause or not. 2 is the
                                     ;; current number of arguments without
                                     ;; including the body. One being the
                                     ;; form itself and the second being the
                                     ;; list of values in the paranthesis.
                                     (if (> ,form-length 2)
                                         ;; If body is provided, call it
                                         ;; with the current offseted value
                                         ;; of pose (for retaining the loop
                                         ;; value on nested calls).
                                         (progn ,@body)
                                         ;; else make the ik-service call
                                         (call-ik-service
                                          offseted-goal-pose
                                          base-link-evaled
                                          tip-link-evaled
                                          seed-state-msg-evaled))
                                   ;; now we either got a solution from the
                                   ;; ik service call, if we're leaf,
                                   ;; or from our nested form.
                                   ;; it can also be that we get NIL back

                                   ;; if we did get an answer,
                                   ;; first, perform the collision check,
                                   ;; and if that is successful, break the loop
                                   (when solution-msg
                                     ;; if collision check is not defined
                                     ;; or the collision check is successful
                                     ;; break the loop
                                     ;; otherwise, we simply continue our loop
                                     (let ((result-joint-values
                                             (cons
                                              (cons ,resampling-axis value)
                                              joint-values)))
                                       (when (or (not solution-valid-p-evaled)
                                                 (funcall solution-valid-p-evaled
                                                          solution-msg
                                                          new-joint-values))
                                         (return (list solution-msg
                                                       result-joint-values)))))))))
                 (setf offseted-goal-pose original-goal-pose)
                 (values (first result) (second result))))))
       (unwind-protect
            (progn
              (roslisp:set-debug-level nil 9)
              ,@body)
         (roslisp:set-debug-level nil old-debug-level)))))


(defmacro find-joint-values-for ((solution-valid-p)
                                 &body body)
  "Method to find joint-values by resamping and checking against
a validation function
Syntax:
 (ik:find-joint-values-for (validation-function)
  (ik::with-resampling (resampling-axis upper-limit
                        lower-limit resampling-step)
   (ik::with-resampling (resampling-axis2 ...)
                            ....))
Resampling axis can only be :X, :Y or :Z"
  `(let ((solution-valid-p-evaled ,solution-valid-p)
         (old-debug-level (roslisp:debug-level nil))
         (new-joint-values))
     (macrolet
         ((with-resampling ((resampling-axis upper-limit lower-limit resampling-step)
                            &body body)
            ;; Formulating a list of joint values to sample
            `(let ((sampling-values
                     (get-joint-values-to-sample
                      ,lower-limit ,upper-limit ,resampling-step)))

               (loop for value in sampling-values
                     do (if (assoc ,resampling-axis new-joint-values)
                            (setf (cdr (assoc ,resampling-axis new-joint-values))
                                  value)
                            (setf new-joint-values
                                  (cons (cons ,resampling-axis value)
                                        new-joint-values)))
                        (let ((result-joint-values
                                (cons (cons ,resampling-axis value)
                                      (progn ,@body))))
                          ;; Return the joint values when the
                          ;; validation function passes, else
                          ;; continue the loop.
                          (when (funcall solution-valid-p-evaled new-joint-values)
                            (return result-joint-values)))))))

       (unwind-protect
            (progn
              (roslisp:set-debug-level nil 9)
              ,@body)
         (roslisp:set-debug-level nil old-debug-level)))))



#+below-is-stuff-for-caching
((defun arm-pose-hash-code (arm-pose-list)
   (let* ((pose (second arm-pose-list))
          (pose-list (cram-tf:pose->flat-list pose))
          (sum (abs (apply #'+ pose-list)))
          (sum-big-precise-num (* sum 100000000))
          (pose-hash-code (floor sum-big-precise-num))
          (arm (first arm-pose-list))
          (overall-hash-code (ecase arm
                               (:left (+ pose-hash-code 10000))
                               (:right (+ pose-hash-code 20000)))))
     overall-hash-code))
 (defun arm-poses-equal-accurate (arm-pose-list-1 arm-pose-list-2)
   (let* ((pose-1 (second arm-pose-list-1))
          (pose-2 (second arm-pose-list-2))
          (arm-1 (first arm-pose-list-1))
          (arm-2 (first arm-pose-list-2)))
     (if (eql arm-1 arm-2)
         (cram-tf:poses-equal-p pose-1 pose-2 0.000001d0 0.000010d0)
         nil)))
 (sb-ext:define-hash-table-test arm-poses-equal-accurate arm-pose-hash-code)
 (defvar *ik-solution-cache* (make-hash-table :test 'arm-poses-equal-accurate))
 (setf (gethash (list left-or-right cartesian-pose) *ik-solution-cache*)
       (list ik-solution-position resulting-torso-angle))
 (let ((hashed-result
         (gethash (list left-or-right cartesian-pose) *ik-solution-cache*)))
   (if hashed-result
       (values (first hashed-result) (second hashed-result))
       (call-ik-stuff))))
