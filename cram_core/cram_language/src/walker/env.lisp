;;;
;;; Copyright (c) 2009, Nikolaus Demmel <demmeln@cs.tum.edu>
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

(in-package :walker)

(defun lambda-list-keyword-p (obj)
  (member obj lambda-list-keywords))

;; TODO: Maybe also pick up declarations in the lexical environment At the
;;       moment there seems to be no need for them.

(defun aug-env (env kind bindings)
  "Returns a new environment that is like env, with the according variable,
   macro, symbol-macro or function bindings added. Other than CLTL2's
   augment-environment, this is designed to be especially convenient to call
   from within the walker."
  (cond ((eq kind :variable)
         (augment-environment
          env
          :variable (mapcar (lambda (x)
                              (if (consp x)
                                  (car x)
                                  x))
                            (remove-if #'lambda-list-keyword-p
                                       bindings))))
        ((eq kind 'macrolet)
         (augment-environment env
                              :macro (mapcar (rcurry #'parse-macro-function env)
                                             bindings)))
        ((eq kind 'symbol-macrolet)
         (augment-environment env :symbol-macro bindings))
        (t
         (augment-environment env :function (mapcar #'car
                                                    bindings)))))

(defun parse-macro-function (form env)
  "Combines enclose and parse-macro, to return something that can be passed to
   augment-environment."
  (list (car form)
        (enclose (parse-macro (car form)
                              (cadr form)
                              (cddr form)
                              env)
                 env)))



;; CLTL2 Environment Access

(defvar *cltl2-environment-fns* nil
  "The functions for augmenting the environment as defined in CLTL2 are not in
the ANSI Standard and thus implementation-spefic.  This lists the functions
needed and is assigned in env-impl-specific.lisp.")

;; I guess you could abstract the next 3 functions and write a general macro,
;; but now there allready written and not likely to change a lot.

(defun augment-environment (env &key variable symbol-macro
                            function macro declare)
  "Calls the implementation specific CLTL2 function."
  (let* ((fns *cltl2-environment-fns*)
         (f (and fns (cdr (assoc :augment-environment fns)))))
    (if (or (null fns)
            (not (fboundp f)))
        (error "Environment access not possible. Function ~a not found." f)
        (funcall (symbol-function f) env
                 :variable variable
                 :symbol-macro symbol-macro
                 :function function
                 :macro macro
                 :declare declare))))

(defun parse-macro (name lambda-list body &optional env)
  "Calls the implementation specific CLTL2 function."
  (let* ((fns *cltl2-environment-fns*)
         (f (and fns (cdr (assoc :parse-macro fns)))))
    (if (or (null fns)
            (not (fboundp f)))
        (error "Environment access not possible. Function ~a not found." f)
        (funcall (symbol-function f) name lambda-list body env))))

(defun enclose (lambda-expression &optional env)
  "Calls the implementation specific CLTL2 function."
  (let* ((fns *cltl2-environment-fns*)
         (f (and fns (cdr (assoc :enclose fns)))))
    (if (or (null fns)
            (not (fboundp f)))
        (error "Environment access not possible. Function ~a not found." f)
        (funcall (symbol-function f) lambda-expression env))))
