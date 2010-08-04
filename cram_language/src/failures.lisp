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

(defvar *break-on-plan-failures* nil
  "When t, invoke the debugger on plan failures. Otherwise, they are
  just passed up to the parent task.")

;;; Condition to cause a plan failure without invoking the debugger.
(define-condition plan-error (error) ())
(define-condition simple-plan-error (simple-condition plan-error) ())

(define-condition composite-failure (plan-error)
  ((failures :initform nil :initarg :failures :reader composite-failures)))

(define-condition rethrown-error (error)
  ((error :initarg :error :initform nil :reader rethrown-error)))

(declaim (inline fail))
(defun fail (&rest args)
  "Like error but throws a simple-plan-error per default."
  ;; Todo: Really behave like error. I guess only some cases are handled here.
  (if args
      (typecase (car args)
        (error (error (car args)))
        (symbol (error (apply #'make-condition (car args) (cdr args))))
        (t (error (make-condition 'simple-plan-error
                                  :format-control (car args)
                                  :format-arguments (cadr args)))))
      (error (make-condition 'simple-plan-error :format-control "Plan failure."))))


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
                    ((rethrown-error (lambda (condition)
                                       (typecase (rethrown-error condition)
                                         ,@(mapcar (lambda (err-def)
                                                     `(,(car err-def) (,(cdr err-def) (rethrown-error condition))))
                                                   condition-handler-syms))))
                     ,@(mapcar (lambda (clause)
                                 `(,(car clause) #',(cdr (assoc (car clause) condition-handler-syms))))
                               clauses))
                  (return (progn ,@body))))))))))
