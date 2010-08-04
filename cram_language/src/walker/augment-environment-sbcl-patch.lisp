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

;; Patch from the sbcl devel mailing list
(eval-when (:compile-toplevel :load-toplevel :execute)
  (require 'sb-cltl2))

(in-package :sb-cltl2)

;; Only use patch in SBCL and if AUGMENT-ENVIRONMENT is not defined.
#+(not #.(cram-utilities:function-bound-feature "AUGMENT-ENVIRONMENT" "SB-CLTL2"))
(progn

  (defvar *null-lexenv* (make-null-lexenv))

  (defun augment-environment
      (env &key variable symbol-macro function macro declare)
    "Create a new lexical environment by augmenting ENV with new information.

   VARIABLE is a list of symbols to introduce as new variable bindings,
   SYMBOL-MACRO is a list symbol macro bindings of the form (name definition)
   MACRO is a list of macro definitions of the form (name definition), where
     definition is a function of two arguments (a form and an environment)
   FUNCTION is a list of symbols to introduce as new local function bindings
   DECLARE is a list of declaration specifiers.  Declaration specifiers
     attach to the new variable or function bindings as if they appeared
     in  let, let*, flet or labels form.  For example
     (augment-environment env :variable '(x) :declare '((special x)))
     is like (let (x) (declare (special x)) ....)
     but (augment-environment (augment-environment env :variable '(x))
                              :declare '((special x)))
     is like (let (x) (locally (declare (special x))) ...) "
    (collect ((lvars)
              (clambdas))
      (unless (or variable symbol-macro function macro declare)
        (return-from augment-environment env))

      (if (null env)
          (setq env (make-null-lexenv))
          (setq env (copy-structure env)))

      ;; a null policy is used to identify a null lexenv
      (when (sb-c::null-lexenv-p env)
        (setf (sb-c::lexenv-%policy env) sb-c::*policy*))

      (when macro
        (setf (sb-c::lexenv-funs env)
              (nconc
               (loop for (name def) in macro
                  collect (cons name (cons 'sb-sys::macro def)))
               (sb-c::lexenv-funs env))))

      (when symbol-macro
        (setf (sb-c::lexenv-vars env)
              (nconc
               (loop for (name def) in symbol-macro
                  collect (cons name (cons 'sb-sys::macro def)))
               (sb-c::lexenv-vars env))))

      (dolist (name variable)
        (lvars (sb-c::make-lambda-var :%source-name name)))

      (dolist (name function)
        (clambdas
         (sb-c::make-lambda
          :lexenv *null-lexenv*
          :%source-name name
          :allow-instrumenting nil)))

      (when declare
        ;; process-decls looks in *lexenv* policy to decide what warnings to print
        (let ((*lexenv* *null-lexenv*))
          (setq env (sb-c::process-decls
                     (list `(declare ,@declare))
                     (lvars) (clambdas) :lexenv env :context nil))))

      (when function
        (setf (sb-c::lexenv-funs env)
              (nconc
               (loop for name in function for lambda in (clambdas)
                  collect (cons name lambda))
               (sb-c::lexenv-funs env))))

      (when variable
        (setf (sb-c::lexenv-vars env)
              (nconc
               (loop for name in variable for lvar in (lvars)
                  collect
                  (cons name
                        ;; if one of the lvars is declared special then process-decls
                        ;; will set it's specvar.
                        (if (sb-c::lambda-var-specvar lvar)
                            (sb-c::lambda-var-specvar lvar)
                            lvar)))
               (sb-c::lexenv-vars env))))

      env)))
