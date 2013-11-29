;; Helpers that define a single GSL function interface
;; Liam Healy 2009-01-07 22:02:20EST defmfun-single.lisp
;; Time-stamp: <2010-06-27 18:28:23EDT defmfun-single.lisp>
;;
;; Copyright 2009 Liam M. Healy
;; Distributed under the terms of the GNU General Public License
;;
;; This program is free software: you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :gsl)

(defun defgeneric-method-p (name)
  "When defining :method in a defgeneric, (nil foo) is used for
   the name, and foo will be returned from this function."
  (if (and (listp name) (null (first name)))
      (second name)))

(defun stupid-code-walk-find-variables (sexp)
  "This will work with the simplest s-expression forms only to find
   variables used."
  (labels ((scwfv (sexp)
	     (if (atom sexp)
		 (when (and (symbolp sexp)
			    (not (eql (symbol-package sexp)
				      (find-package :keyword))))
		   (list sexp))
		 (unless (member (first sexp) '(quote))
		   (mappend #'scwfv (rest sexp))))))
    (remove nil (remove-duplicates (scwfv sexp)))))

(defun variables-used-in-c-arguments (c-arguments)
  "Find the arguments passed to the C function.  This is a poor
  quality code walker, but is sufficient for actual usage of defmfun."
  (remove-duplicates
   (mappend (lambda (carg)
	     (stupid-code-walk-find-variables (grid:st-symbol carg)))
	   c-arguments)
   :from-end t))

(define-condition obsolete-gsl-version (error)
  ((name :initarg :name :reader name)
   (gsl-name :initarg :gsl-name :reader gsl-name)
   (gsl-version :initarg :gsl-version :reader gsl-version))
  (:report
   (lambda (condition stream)
     (apply 'format stream
	    "Function ~a (~a) is not available in the ~
             currently loaded release ~a of GSL; it was introduced ~
             in release ~d.~d.  Once you have upgraded GSL, you will ~
             need to delete fasl files for GSLL and recompile it."
	    (name condition)
	    (gsl-name condition)
	    *gsl-version*
	    (gsl-version condition))))
  (:documentation
   "An error indicating that the currently loaded version of the GSL libary
    does not have the function defined."))

(defun complete-definition
    (defn name arglist gsl-name c-arguments key-args
     &optional
     (body-maker 'body-no-optional-arg)
     (mapdown (eq body-maker 'body-optional-arg)))
  "A complete definition form, starting with defun, :method, or defmethod."
  (with-defmfun-key-args key-args
    (if (have-at-least-gsl-version gsl-version)
	`(,defn
	     ,@(when (and name (not (defgeneric-method-p name)))
		     (list name))
	     ,@(when qualifier (list qualifier))
	   ,arglist
	   ,(declaration-form
	     (cl-argument-types arglist c-arguments)
	     (set-difference	       ; find all the unused variables
	      (arglist-plain-and-categories arglist nil)
	      (remove-duplicates
	       (union
		(if mapdown
		    (apply 'union
			   (mapcar 'variables-used-in-c-arguments c-arguments))
		    (variables-used-in-c-arguments c-arguments))
		;; Forms in :before, :after are checked for used variables
		(stupid-code-walk-find-variables
		 (cons
		  'values
		  (append before after
			  (callback-symbol-set
			   callback-dynamic cbinfo (first callback-dynamic-variables))
			  ;; &optional/&key/&aux defaults are checked
			  (let ((auxstart (after-llk arglist)))
			    (when auxstart
			      (apply
			       'append
			       (mapcar 'rest (remove-if 'atom auxstart)))))))))))
	     (first callback-dynamic-variables))
	   ,@(when documentation (list documentation))
	   ,(funcall body-maker name arglist gsl-name c-arguments key-args))
	`(,defn
	     ,@(when (and name (not (defgeneric-method-p name)))
		     (list name))
	     ,@(when qualifier (list qualifier))
	   ,arglist
	   (declare (ignorable ,@(arglist-plain-and-categories arglist nil)))
	   (error 'obsolete-gsl-version
		  :name ',name :gsl-name ',gsl-name :gsl-version ',gsl-version)))))

(defun wrap-letlike (when binding wrapping body)
  (if when
      `(,wrapping ,binding ,@body)
      (if (eql (length body) 1)
	  (first body)
	  `(progn ,@body))))

(defun body-no-optional-arg (name arglist gsl-name c-arguments key-args)
  "Wrap necessary array-handling forms around the expanded unswitched
  body form."
  ;; With static-vectors, nothing is now necessary, so this just calls body-expand.
  (body-expand name arglist gsl-name c-arguments key-args))
