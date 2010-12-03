;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;


(in-package :cpl-impl)

(define-task-variable *break-on-plan-failures* nil
  "Like *BREAK-ON-SIGNALS*, but for plan failures."
  :type (or symbol list))               ; type-specifier

(define-condition plan-failure (serious-condition) ()
  (:documentation
   "Condition which denotes a plan failure."))

(define-condition simple-plan-failure (simple-condition plan-failure) ())

(define-condition composite-failure (plan-failure)
  ((failures :initform nil :initarg :failures :reader composite-failures)))

(defun coerce-to-condition (datum arguments default-type)
  (etypecase datum
    (condition (prog1 datum (assert (null arguments))))
    (symbol    (apply #'make-condition datum arguments))
    (string    (make-condition default-type
                               :format-control datum
                               :format-arguments arguments))))

(defun %fail (datum args)
  (let ((current-task *current-task*)
        (condition (coerce-to-condition datum args 'simple-plan-failure)))
    (log-event
      (:context "FAIL")
      (:display "~S: ~_\"~A\"" condition condition)
      (:tags :fail))
    (cond
      ;; The main thread will perform JOIN-TASK on the TOPLEVEL-TASK,
      ;; signal the condition for that case.
      ((not current-task)
       (error condition))
      ;; The condition does not designate a plan failure so just
      ;; signal it, entering the debugger if it remains unhandled.
      ((not (typep condition 'plan-failure))
       (error condition))
      ;; Does the user want us to enter the debugger on the plan
      ;; failure?
      ((and (typep condition *break-on-plan-failures*)
            (restart-case
                (with-condition-restarts condition
                    (list (find-restart 'propagate))
                  (without-scheduling
                    (invoke-debugger condition)))
              (continue ()
                :report "Continue failure signalling."
                nil))))                 ; fall through COND clause
      ;; An actual plan failure. Use SIGNAL here, too, so
      ;; WITH-FAILURE-HANDLING can be implemented on top of the normal
      ;; CL condition system.
      ;;
      ;; At the moment, ASSERT-NO-RETURNING, but in case inter-task
      ;; restarts are implemented, this has to change.
      (t
       (assert-no-returning
         (signal condition))))))

(defun fail (&rest args)
  (if (null args)
      (%fail "Plan failure." nil)
      (%fail (car args) (cdr args))))


(defmacro with-failure-handling (clauses &body body)
  "Macro that replaces handler-case in cram-language. This is
necessary because error handling does not work across multiple
threads. When an error is signaled, it is put into an envelope to
avoid invocation of the debugger multiple times. When handling errors,
this envelope must also be taken into account.

We also need a mechanism to retry since errors can be caused by plan
execution and the environment is highly non-deterministic. Therefore,
it is possible to use the function `retry' that is lexically bound
within with-failure-handling and causes a re-execution of the body.

When an error is unhandled, it is passed up to the next failure
handling form (exactly like handler-bind). Errors are handled by
invoking the retry function or by doing a non-local exit. Note that
with-failure-handling implicitly creates an unnamed block,
i.e. `return' can be used."
  (with-gensyms (wfh-block-name)
    (let ((condition-handler-syms (loop for clause in clauses
                                     collecting (cons (car clause)
                                                      (gensym (symbol-name (car clause)))))))
      `(block nil
         (tagbody ,wfh-block-name
            (flet ((retry ()
                     (go ,wfh-block-name)))
              (declare (ignorable (function retry)))
              (flet ,(mapcar (lambda (clause)
                               (destructuring-bind (condition-name lambda-list &rest body)
                                   clause
                                 `(,(cdr (assoc condition-name condition-handler-syms)) ,lambda-list
                                    ,@body)))
                             clauses)
                (handler-bind
                    ,(mapcar (lambda (clause)
                               `(,(car clause) #',(cdr (assoc (car clause) condition-handler-syms))))
                             clauses)
                  (return (progn ,@body))))))))))
