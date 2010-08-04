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

(in-package :liswip)

(defvar *swi-predicates* nil)
(defvar *prolog-module* nil)

(defvar *swi-predicates-results* (make-data-pool))

(defstruct (swi-predicate (:conc-name nil))
  name
  callback-symbol
  arity
  module
  (registered nil))

(defmacro in-prolog-module (name)
  `(eval-when (:compile-toplevel :load-toplevel :execute)
     (setf *prolog-module* (string ,name))))

(defun declare-swi-predicate (callback name arity &optional
                           (module *prolog-module*))
  (pushnew (make-swi-predicate :name name :callback-symbol callback
                                :arity arity :module module)
           *swi-predicates* :key #'callback-symbol))

(defun register-swi-predicates ()
  (when (liswip-initialized)
    (loop for swi-predicate in *swi-predicates*
       unless (registered swi-predicate) do
       (if (module swi-predicate)
           (pl-register-foreign-in-module (module swi-predicate)
                                          (name swi-predicate)
                                          (arity swi-predicate)
                                          (get-callback (callback-symbol swi-predicate))
                                          (logior PL_FA_VARARGS PL_FA_NONDETERMINISTIC))
           (pl-register-foreign (name swi-predicate) (arity swi-predicate)
                                (get-callback (callback-symbol swi-predicate))
                                (logior PL_FA_VARARGS PL_FA_NONDETERMINISTIC)))
       (setf (registered swi-predicate) t))))

(defmacro def-swi-predicate (name args &body body)
  "Defines a new predicate for SWI Prolog.
   `name' is either a symbol specifying the name of the predicate or a
   list of the form (lisp-name prolog-name) with lisp-name being a
   symbol and prolog-name being a string specifying the name of the
   predicate as used in prolog.  Please note that in the first
   version, the name is pre-processed to generate the prolog
   name. That means '-' is converted to '_' and the predicate will be
   lower case.  

   `args' is a normal lambda list and `body' is the body of the
   predicate.  The return value must be a (lazy) list of solutions
   where a solution is a list of conses with the variable in its car
   and the value in its cdr."
  (let* ((arity (list-length args))
         (function-name (let ((name (if (listp name) (car name) name)))
                          (intern (format nil (format nil "~a/~a"
                                                      (symbol-name name)
                                                      arity))
                                  (symbol-package name))))
         (prolog-name (if (listp name)
                          (cadr name)
                          (prologify name))))
    `(progn
       (defun ,function-name ,args
         ,@body)
       (defcallback ,function-name foreign_t ((t-0 term_t)
                                              (arity :int)
                                              (control control_t))
         (flet ((marshall-bdgs (bdgs terms decoded-term)
                  (loop for out-term from terms
                     for term in decoded-term
                     when (is-var term)
                     do (swi-unify out-term (var-value term bdgs)))))
           (let ((decoded-term (decode-term t-0 arity))
                 (control-flag (pl-foreign-control control)))
             (cond ((eql control-flag PL-FIRST-CALL)
                    (let* ((bdgs (apply #',function-name decoded-term))
                           (new-pool-handle
                            (when bdgs
                              (new-pool-value *swi-predicates-results* (lazy-cdr bdgs)))))
                      (cond (bdgs
                             (marshall-bdgs (lazy-car bdgs) t-0 decoded-term)
                             (pl-retry new-pool-handle))
                            (t
                             FALSE))))
                   ((eql control-flag PL-REDO)
                    (let* ((pool-handle (pl-foreign-context control))
                           (pool-value (pool-value *swi-predicates-results* pool-handle)))
                      (cond (pool-value
                             (setf (pool-value *swi-predicates-results* pool-handle)
                                   (lazy-cdr pool-value))
                             (marshall-bdgs (lazy-car pool-value) t-0 decoded-term)
                             (pl-retry pool-handle))
                            (t
                             (delete-pool-value *swi-predicates-results* pool-handle)
                             FALSE))))
                   ((eql control-flag PL-CUTTED)
                    (let ((pool-handle (pl-foreign-context control)))
                      (delete-pool-value *swi-predicates-results* pool-handle)                      
                      TRUE))))))
       (eval-when (:load-toplevel :execute)
         (declare-swi-predicate ',function-name ,prolog-name ,arity))
       (eval-when (:load-toplevel :execute)
         (register-swi-predicates)))))
