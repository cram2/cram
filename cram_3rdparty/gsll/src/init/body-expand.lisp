;; Expand the body of a defmfun
;; Liam Healy 2009-04-13 22:07:13EDT body-expand.lisp
;; Time-stamp: <2011-10-30 00:35:29EDT body-expand.lisp>
;;
;; Copyright 2009, 2010, 2011 Liam M. Healy
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

(defun creturn-st (c-return)
  "The symbol-type of the return from the C function."
  (let ((supplied-symbol-p		; return symbol supplied
	  (and (listp c-return)		; accommodate (:struct foo) type spec
	       (not (eq (symbol-package (first c-return)) (find-package :keyword))))))
    (grid:make-st
     (if supplied-symbol-p
      (grid:st-symbol c-return)
      (make-symbol "CRETURN"))
     (if (member c-return *special-c-return*)
	 :int
	 (if supplied-symbol-p (grid:st-type c-return) c-return)))))

(defun cl-convert-form (decl)
  "Generate a form that calls the appropriate converter from C/GSL to CL."
  (list `(cffi:mem-ref ,(grid:st-symbol decl) ',(grid:st-actual-type decl))))

(defun values-unless-singleton (forms)
  (unless (listp forms) (error "Values are not a list."))
  (if (rest forms)
    `(values ,@forms)
    (first forms)))

(defun body-expand (name arglist gsl-name c-arguments key-args)
  "Expand the body (computational part) of the defmfun."
  (with-defmfun-key-args key-args
    (let* ((creturn-st (creturn-st c-return))
	   (allocated-return ; Allocated and then returned from CL function
	     (mapcar
	      (lambda (s)
		(or (find s c-arguments :key #'grid:st-symbol)
		    ;; Catch programming errors, usually typos
		    (error "Could not find ~a among the arguments" s)))
	      (remove-if
	       (lambda (s)
		 (member s (arglist-plain-and-categories arglist nil)))
	       (variables-used-in-c-arguments c-arguments))))
	   (clret (or			; better as a symbol macro
		   (substitute
		    (grid:st-symbol creturn-st) :c-return
		    (mapcar (lambda (sym)
			      (let ((it (find sym allocated-return :key 'grid:st-symbol)))
				(if it
				    (first (cl-convert-form it))
				    sym)))
			    return))
		   (mappend
		    #'cl-convert-form
		    (callback-remove-arg allocated-return cbinfo 'grid:st-symbol))
		   outputs
		   (unless (eq c-return :void)
		     (list (grid:st-symbol creturn-st))))))
      (wrap-letlike
       allocated-return
       (mapcar (lambda (d) (wfo-declare d cbinfo))
	       allocated-return)
       'cffi:with-foreign-objects
       `(,@(append
	    (callback-symbol-set
	     callback-dynamic cbinfo (first callback-dynamic-variables))
	    before
	    (when callback-object (callback-set-dynamic callback-object arglist)))
	 ,@(callback-set-slots
	    cbinfo callback-dynamic-variables callback-dynamic)
	 (let ((,(grid:st-symbol creturn-st)
		 (cffi:foreign-funcall
		  ,gsl-name
		  ,@(append
		     (mappend
		      (lambda (arg)
			(list (cond
				((member (grid:st-symbol arg)
					 allocated-return)
				 :pointer)
				(t (grid:st-type arg)))
			      (grid:st-symbol arg)))
		      (mapcar 'grid:st-pointer-generic-pointer
			      c-arguments))
		     (list (grid:st-type creturn-st))))))
	   ,@(case c-return
	       (:void `((declare (ignore ,(grid:st-symbol creturn-st)))))
	       (:error-code		; fill in arguments
		`((check-gsl-status ,(grid:st-symbol creturn-st)
				    ',(or (defgeneric-method-p name) name)))))
	   ,@(when (eq (grid:st-type creturn-st) :pointer)
	       `((check-null-pointer
		  ,(grid:st-symbol creturn-st)
		  ,@'('memory-allocation-failure "No memory allocated."))))
	   ,@after
	   ,(values-unless-singleton
	     (defmfun-return
		 c-return (grid:st-symbol creturn-st) clret
	       allocated-return
	       return return-supplied-p
	       enumeration outputs))))))))

(defun defmfun-return
    (c-return cret-name clret allocated return return-supplied-p enumeration outputs)
  "Generate the return computation expression for defmfun."
  (case c-return
    (:number-of-answers
     (mappend
      (lambda (vbl seq)
	`((when (> ,cret-name ,seq) ,vbl)))
      clret
      (loop for i below (length clret) collect i)))
    (:success-failure
     (if (equal clret outputs)
	 ;; success-failure more important than passed-in
	 `((success-failure ,cret-name))
	 (remove cret-name		; don't return c-return itself
		 `(,@clret (success-failure ,cret-name)))))
    (:success-continue
     (if (equal clret outputs)
	 ;; success-failure more important than passed-in
	 `((success-continue ,cret-name))
	 (remove cret-name		; don't return c-return itself
		 `(,@clret (success-continue ,cret-name)))))
    (:true-false
     `((not (zerop ,cret-name))))
    (:enumerate
     `((cffi:foreign-enum-keyword ',enumeration ,cret-name)))
    (t (unless
	   (or
	    (and (eq c-return :error-code)
		 (not outputs)
		 (not allocated)
		 (not return-supplied-p))
	    (and (null return) return-supplied-p))
	 clret))))
