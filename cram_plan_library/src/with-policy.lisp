;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;       Universität Bremen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :cram-plan-library)

;; Example on how to use policies (for function semantics, see the
;; respective function doc strings):
;;
;; (define-policy my-policy (max-num match-num)
;;   (:init (format t "Initializing policy~%")
;;          t)
;;   (:check (format t "Checking if random number from 0
;;                      to ~a equals ~a~%" max-num match-num)
;;           (let ((rnd (random max-num)))
;;             (format t "Got number ~a~%" rnd)
;;             (cond ((eql rnd match-num)
;;                    (format t "Match~%")
;;                    t)
;;                   (t (sleep 1)))))
;;   (:recover (format t "Running recovery mechanisms~%"))
;;   (:clean-up (format t "Running clean-up~%")))
;;
;; (top-level
;;   (with-named-policy 'my-policy (10 5)
;;     (loop do (format t "Main loop cycle.~%")
;;              (sleep 2))))

(defclass policy ()
  ((name :reader name :initarg :name)
   (parameters :reader parameters :initarg :parameters)
   (description :reader description :initarg :description)
   (init :reader init :initarg :init)
   (check :reader check :initarg :check)
   (recover :reader recover :initarg :recover)
   (clean-up :reader clean-up :initarg :clean-up)))

(defvar *policies* nil "List of defined policies")

(define-condition policy-not-found () ())
(define-condition policy-init-failed () ())

(defmacro make-policy (name parameters &rest properties)
  "Generates a policy based on the information supplied. `name'
specifies an internal name for the policy, to be used with
`named-policy', or `with-named-policy'. `parameters' is a list of
parameter symbols to be used by code inside the policy. Every time, a
piece of code inside the policy is executed, these parameters (with
assigned values from the `with-policy', or `with-named-policy' call)
are passed to the code segments. The `properties' variable holds a
list of labelled code segments for execution during certain phases. An
example would look like this:

> (make-policy
    policy-1 (param-1 param-2)
    (:init (do-initialization-here))
    (:check (do-checking-here))
    (:recover (do-recovering-here))
    (:clean-up (do-cleaning-up-here))
    (:description \"The description string\")

This returns a policy object to be used with `with-policy'. For
further information about when each function block is executed, see
`with-policy'. The semantics of the `properties' variable are like
this:

- Forms given via `:init' are executed exactly once, when the policy
  is initialized for usage. In case this function returns `nil',
  execution of the `body' code is not started, none of the other
  policy code blocks are executed, and a failure of type
  `policy-init-failed' is thrown.

- A function given under the label `:check' is executed every time
  `with-policy' checks if the policy condition is met or not. If this
  function returns `t', the condition is met, the `:recover' function
  block is executed, and the execution of both, the policy, and the
  wrapped body code of `with-policy' is stopped.

- Either when the policy (due to a met policy condition), or the
  wrapped body of `with-policy' code stopped execution of the current
  code block, the `:clean-up' function block is executed to perform
  clean-up procedures.

- An optional `:description' string can be given to identify the
  meaning of the policy."
  (let* ((block-identifiers `(:init :check :recover :clean-up)))
    `(make-instance 'policy
                    :name ',name
                    :parameters ',parameters
                    :description ',(second
                                    (find ':description
                                          properties
                                          :test (lambda (x y)
                                                  (eql x (first y)))))
                    ,@(loop for identifier in block-identifiers
                            for prop = (rest (find
                                              identifier properties
                                              :test (lambda (x y)
                                                      (eql x (first y)))))
                            collect identifier
                            collect (when prop
                                      `(lambda ,parameters ,@prop))))))

(defmacro define-policy (name parameters &rest properties)
  "This macro implicitly calls `make-policy', and pushes the generated
policy onto the list of defined policies, thus making it accessible to
`named-policy' and `with-named-policy' by its name. The usage is the
same as for `make-policy':

> (define-policy
    policy-1 (param-1 param-2)
    (:init (do-initialization-here))
    (:check (do-checking-here))
    (:recover (do-recovering-here))
    (:clean-up (do-cleaning-up-here)))"
  `(progn
     (setf *policies*
           (remove ',name *policies*
                   :test (lambda (x y)
                           (eql x (name y)))))
     (let ((new-policy (make-policy ,name ,parameters ,@properties)))
       (push new-policy *policies*)
       new-policy)))

(defun named-policy (policy-name)
  "Returns the policy by the name `policy-name' from the list of
defined policies. If the policy by this name is not in the list, the
`policy-not-found' condition is signalled. Usage:

> (named-policy 'policy-name)"
  (let ((policy (find policy-name *policies*
                      :test (lambda (x y) (eql x (name y))))))
    (cond (policy policy)
          (t (fail 'policy-not-found)))))

(defmacro with-named-policy (policy-name policy-parameters &body body)
  "Performs the same as `with-policy', but accepts a policy name
instead of the policy object itself. This calls an implicit
`named-policy' to acquire the policy object. Otherwise, it has the
same semantics as `with-policy'. Usage:

> (with-named-policy 'policy-name (param-value-1 param-value-2)
    (body-code))"
  (let ((policy `(named-policy ,policy-name)))
    `(with-policy ,policy ,policy-parameters ,@body)))

(defmacro with-policy (policy policy-parameters &body body)
  "Wraps the code given as `body' into a `pursue' construct together
with monitoring code supplied by the policy `policy', and given the
parameters `policy-parameters'. The `policy-parameters' allow for
custom parameterization of policies. First, the policy is initialized
via the optional `:init' code block. In case this block returns `nil',
execution of the `body' code or other policy-related code blocks is
not started. An exception of type `policy-init-failed' is
thrown. Otherwise, the `:check' code block of the policy is executed
in a loop in parallel to the `body' code. If the `:check' code returns
`t', the policy condition is met and the `:recover' code block is
executed. The execution of both, the policy, and the `body' code is
the stopped, and the `:clean-up' policy code is executed. If the
policy condition is never met, `body' finishes and returns
normally.

To clarify the order of code execution here:

- Initialization of policy is executed (`:init')

- `pursue' code form is started, with up to two forms inside:
  - The policy `:check' code block (if present)
  - The `body' code

- `:check' is evaluated continuously, in parallel to the normal
  execution of `body'. If it returns `nil', nothing happens. In any
  other case (i.e. return value is unequal to `nil'), the execution of
  the `body' code is interrupted, and `:check' is not performed again
  anymore. The policy code block given in `:recover' is executed (if
  present). This means (explicitly) that the `:recover' code is
  performed *after* the `body' code got interrupted.

- If `:check' always returns `nil' until the `body' code execution
  finishes, `:recover' is never executed.

- In either case (with or without `:recover'), the policy `:clean-up'
  code is performed (if present).

Usage:

> (with-policy policy-object (param-value-1 param-value-2)
    (body-code))"
  (let ((init `(init ,policy))
        (check `(check ,policy))
        (clean-up `(clean-up ,policy))
        (recover `(recover ,policy)))
    `(progn
       (when ,init
         (unless (funcall ,init ,@policy-parameters)
           (cpl:fail 'policy-init-failed)))
       (let ((flag-do-recovery nil))
         (unwind-protect
              (pursue
                (when ,check
                  (loop while (not (funcall ,check ,@policy-parameters)))
                  (setf flag-do-recovery t))
                (progn ,@body))
           (progn
             (when (and ,recover flag-do-recovery)
               (funcall ,recover ,@policy-parameters))
             (when ,clean-up
               (funcall ,clean-up ,@policy-parameters))))))))
