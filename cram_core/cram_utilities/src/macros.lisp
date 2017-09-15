;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
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

(in-package :cram-utilities)

(defmacro string-case (var &body cases)
  (once-only (var)
    `(cond ,@(mapcar (lambda (case-expr)
                       (destructuring-bind (pat &rest body) case-expr
                         (if (eq pat t)
                             `(t ,@body)
                             `((string-equal ,var ,pat)
                               ,@body))))
              cases))))

;;; From swank.lisp which is public domain.
(defmacro destructure-case (value &rest patterns)
  "Dispatch VALUE to one of PATTERNS.
A cross between `case' and `destructuring-bind'.
The pattern syntax is:
  ((HEAD . ARGS) . BODY)
The list of patterns is searched for a HEAD `eq' to the car of
VALUE. If one is found, the BODY is executed with ARGS bound to the
corresponding values in the CDR of VALUE."
  (let ((operator (gensym "op-"))
        (operands (gensym "rand-"))
        (tmp (gensym "tmp-")))
    `(let* ((,tmp ,value)
            (,operator (car ,tmp))
            (,operands (cdr ,tmp)))
       (case ,operator
         ,@(loop for (pattern . body) in patterns collect 
                 (if (eq pattern t)
                     `(t ,@body)
                     (destructuring-bind (op &rest rands) pattern
                       `(,op (destructuring-bind ,(ensure-list rands)
                                 ,operands 
                               ,@body)))))
         ,@(if (eq (caar (last patterns)) t)
               '()
               `((t (error "destructure-case failed: ~S" ,tmp))))))))

;;; Note: this implementation does not properly handle declarations.
(defmacro flet* (bindings &body body)
  "FLET* is to FLET what LET* is to LET."
  (labels ((gen (bindings body)
             (if (null bindings)
                 body
                 `((flet (,(first bindings))
                      ,@(gen (rest bindings) body))))))
    (car (gen bindings body))))

(defun body-context (forms)
  (let ((*print-length* 3)
        (*print-level* 3)
        (*print-pretty* t))
    (prin1-to-string forms)))

(defun %assert-no-returning (ctx thunk)
  (funcall thunk)
  (error "Asserted~%  ~@<~A~:>~%not to return. But we did return." ctx))

(defmacro assert-no-returning (&body body)
  `(%assert-no-returning ,(body-context body) #'(lambda () ,@body)))

(defun %assert-no-nlx (ctx thunk)
  (unwind-protect-case ()
      (funcall thunk)
    (:abort
     (error "Asserted~%  ~@<~A~:>~%not to perform a non-local ~
             exit. Caught attempt to do so." ctx))))

(defmacro assert-no-nlx (&body body)
  `(%assert-no-nlx ,(body-context body) #'(lambda () ,@body)))
