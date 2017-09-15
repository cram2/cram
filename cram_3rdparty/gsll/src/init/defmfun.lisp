;; Macro for defining GSL functions.
;; Liam Healy 2008-04-16 20:49:50EDT defmfun.lisp
;; Time-stamp: <2016-06-14 23:39:02EDT defmfun.lisp>
;;
;; Copyright 2008, 2009, 2010, 2014, 2016 Liam M. Healy
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

;;;;****************************************************************************
;;;; Macro defmfun
;;;;****************************************************************************

;;; Demfun is the main macro for defining functions (ordinary
;;; functions, generic functions, and methods) that call GSL
;;; functions.  It takes care of mapping the data from CL to C and
;;; then back out from C to CL.  Where the GSL function returns a
;;; condition code, it will insert a check that turns that result into
;;; a CL warning.  For generic functions and methods, it will generate
;;; the interfaces to all the specific GSL functions.

;;; Required arguments to defmfun:
;;; name        The name of the function being defined in CL; if NIL, then a lambda is created.
;;; arglist     The CL argument list for the function
;;; gsl-name    A string or list of strings representing the name(s) of
;;;             the GSL function(s)
;;; c-arguments A list of arguments like (symbol c-type)
;;;             (or list of such lists) to the GSL
;;;             functions, including CFFI declarations.
;;;             Anything not in arglist will be allocated.

;;; Keyword arguments to defmfun:
;;; c-return   A symbol naming a type, (e.g. :int, :double, :void),
;;;            or a list of (symbol type) to name the value,
;;;            or :error-code, :number-of-answers, :success-failure,
;;;            :true-false, :enumerate.  If :enumeration is given,
;;;            the :enumeration keyword argument will supply the name
;;;            of the enumeration.
;;; return     A list of quantities to return.
;;;            May be or include :c-return to include the c-return value
;;;            or its derivatives.
;;;            Default are allocated quantities in c-arguments, or :c-return if none.
;;; definition :function, :generic, :method, :methods
;;; qualifier  A method qualifier such as :after or :before.
;;; element-types
;;;            Permissible types for elements of arrays.  May be
;;;            NIL meaning all of grid:*array-element-types*, :no-complex
;;;            meaning that list without the complex types, 
;;;	       :float meaning only the float types, :complex only
;;;	       the complex types, :float-complex both float and complex,
;;;            or a list of element types.
;;; index      Name under which this function should be cross-referenced; t
;;;            to use name, nil to not index.
;;; export     Whether to export the symbol.
;;; documentation
;;; inputs     Arrays whose values are used by the GSL function.
;;; outputs    Arrays that are written to in the GSL function.
;;; before     Forms to be evaluated before the foreign call.
;;; after      Forms to be evaluated after the foreign call.
;;; enumeration  The name of the enumeration return.
;;; gsl-version  The GSL version at which this function was introduced.
;;; switch     Switch only the listed optional/key variables (default all of them)
;;; callbacks  A list that specifies the callback structure and function(s); see callback.lisp
;;; callback-dynamic Values for callback function(s) set at runtime.  The order matches
;;;            the functions listed in :callbacks.  See callback.lisp for contents.
;;; callback-object  Name of the object has callbacks.

(defmacro defmfun (name arglist gsl-name c-arguments &rest key-args)
  "Definition of a GSL function."
  (expand-defmfun-wrap name arglist gsl-name c-arguments key-args))

;;; Utility for the helper functions
(defmacro with-defmfun-key-args (key-args &body body)
  "Bind defmfun's key arguments."
  `(destructuring-bind
    (&key (c-return :error-code)
     (return nil return-supplied-p)
     element-types
     (index t)
     (definition :function)
     (export (not (member definition (list :method :methods))))
     documentation inputs outputs before after enumeration qualifier
     gsl-version switch ((:callbacks cbinfo)) callback-dynamic callback-object)
    ,key-args
    (declare (ignorable c-return return return-supplied-p definition element-types
      index export documentation inputs outputs
      before after enumeration qualifier
      gsl-version switch cbinfo callback-dynamic callback-object)
     (special indexed-functions callback-dynamic-variables #+fsbv fsbv-functions))
    ,@body))

(defun optional-args-to-switch-gsl-functions (arglist gsl-name)
  "The presence/absence of optional arguments will switch between the
   first and second listed GSL function names."
  (and (intersection *defmfun-optk* arglist)
       (listp gsl-name)
       (or
	(every 'stringp gsl-name)
	(listp (first gsl-name)))))

(defun expand-defmfun-wrap (name arglist gsl-name c-arguments key-args)
  (let (indexed-functions callback-dynamic-variables #+fsbv fsbv-functions)
    ;; workaround for compiler errors that don't see 'indexed-function is used
    (declare (ignorable indexed-functions callback-dynamic-variables #+fsbv fsbv-functions))
    (with-defmfun-key-args key-args
      (setf indexed-functions (list)
	    callback-dynamic-variables
	    ;; A list of variable names, and a list of callback names
	    (when (or cbinfo callback-object)
	      (if callback-object
		  (let ((class
			  (if (listp callback-object)
			      (second (first callback-object))
			      (category-for-argument arglist callback-object))))
		    (list (mobject-fnvnames
			   class
			   (number-of-callbacks (get-callbacks-for-class class)))
			  nil))
		  (let ((num-callbacks (number-of-callbacks cbinfo)))
		    (list
		     (make-symbol-cardinals
		      (list name 'dynfn) num-callbacks :gsl)
		     (make-symbol-cardinals
		      (list name 'cbfn) num-callbacks :gsl)))))
	    #+fsbv fsbv-functions #+fsbv (list))
      (wrap-index-export
       (cond
	 ((eq definition :generic)
	  (expand-defmfun-generic name arglist gsl-name c-arguments key-args))
	 ((eq definition :method)
	  (expand-defmfun-method name arglist gsl-name c-arguments key-args))
	 ((eq definition :methods)
	  (expand-defmfun-defmethods name arglist gsl-name c-arguments key-args))
	 ((optional-args-to-switch-gsl-functions arglist gsl-name)
	  (expand-defmfun-optional name arglist gsl-name c-arguments key-args))
	 ((member definition '(nil :function))
	  (if name
	      (complete-definition 'cl:defun name arglist gsl-name c-arguments key-args)
	      (complete-definition 'cl:lambda nil arglist gsl-name c-arguments key-args))))
       name gsl-name key-args))))

(defun wrap-progn (args)
  "Wrap the arguments in a progn."
  (if (rest args)
      #-clisp (cons 'progn args)
      #+clisp (append (list 'let nil) args) ; CLISP bug workaround
      (first args)))

(defun wrap-index-export (expanded-body name gsl-name key-args)
  "Wrap the expanded-body with index and export if requested.
   Use a progn if needed."
  (with-defmfun-key-args key-args
    (let ((index-export
	    (when name
	      (if (eq index t) (setf index name))
	      (flet ((mapnfn (gslnm) `(map-name ',index ,gslnm)))
		(append
		 (when index
		   (if indexed-functions
		       (mapcar #'mapnfn indexed-functions)
		       (if (listp gsl-name)
			   (mapcar #'mapnfn gsl-name)
			   (list (mapnfn gsl-name)))))
		 (when export `((export ',name))))))))
      (wrap-progn
       (append
	(if (symbolp (first expanded-body)) (list expanded-body) expanded-body)
	(make-defmcallbacks
	 cbinfo
	 (second callback-dynamic-variables)
	 (first callback-dynamic-variables))
	#+fsbv
	fsbv-functions
	index-export)))))

;;;;****************************************************************************
;;;; A method for a generic function, on any class
;;;;****************************************************************************

(defun expand-defmfun-method (name arglist gsl-name c-arguments key-args)
  "Create a specific method for a previously-defined generic function."
  (with-defmfun-key-args key-args
    (if (listp gsl-name)
	(mapc (lambda (n) (push n indexed-functions)) gsl-name)
	(push gsl-name indexed-functions))
    ;;(remf key-args :documentation)
      (complete-definition
       'cl:defmethod
       name
       arglist
       gsl-name
       c-arguments
       key-args
       (if (optional-args-to-switch-gsl-functions arglist gsl-name)
	   'body-optional-arg 'body-no-optional-arg)
       (listp gsl-name))))

;;;;****************************************************************************
;;;; Optional argument(s)
;;;;****************************************************************************

(defun expand-defmfun-optional
    (name arglist gsl-name c-arguments key-args)
  "Expand defmfun where there is an optional argument
   present, giving the choice between two different GSL functions."
  (complete-definition 'cl:defun name arglist gsl-name c-arguments key-args
		       'body-optional-arg))

(defun body-optional-arg
    (name arglist gsl-name c-arguments key-args)
  "Create the body of a defmfun with &optional in its arglist,
   where the presence or absence of the optional argument(s)
   selects one of two GSL functions."
  (let ((optpos (position-if (lambda (s) (member s *defmfun-optk*)) arglist)))
    (if optpos
	(with-defmfun-key-args key-args
	  (let ((mandatory-arglist (subseq arglist 0 optpos))
		(optional-arglist (subseq arglist (1+ optpos))))
	    `(if ,(or (first switch) (first optional-arglist))
		 ,(body-no-optional-arg 
		   name
		   (append mandatory-arglist optional-arglist)
		   (second gsl-name)
		   (second c-arguments)
		   key-args)
		 ,(body-no-optional-arg
		   name
		   (append mandatory-arglist
			   (when switch
			     (remove-if
			      (lambda (arg)
				(member (if (listp arg) (first arg) arg) switch))
			      optional-arglist)))
		   (first gsl-name)
		   (first c-arguments)
		   key-args)))))))
