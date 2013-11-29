;; Foreign callback functions.               
;; Liam Healy 
;; Time-stamp: <2010-07-11 19:01:17EDT callback.lisp>
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

;;; Callback functions are functions which are passed as data; to Lisp
;;; that means they are just functions, but C makes a distinction.
;;; They are needed by several GSL tasks.  Functions that take one
;;; double-float and return one double-float and are defined using the
;;; gsl_function structure (see gsl_math.h) are called scalar
;;; functions.  They are used in numerical-integration,
;;; numerical-differentiation, chebyshev and definitions to aid in
;;; creating and using them are provided here.  The idea behind the
;;; definitions is that they absorb us much of the definition overhead
;;; as possible so that the user just defines a CL function and calls
;;; a macro to generate something to pass to the GSL functions.

;;; Other GSL tasks make use of callback functions with different
;;; characteristics.  Since they are specific to each of the tasks,
;;; they are defined with those tasks.  The macro #'defmcallback can
;;; specify that the CL function is to expect in arglist and return as
;;; multiple values scalar quantities that come from and will be bound
;;; to either grid:foreign-arrays.  This is done with a list of the
;;; type and size, e.g. (:double 3), and for setting :set, type size,
;;; e.g. (:set :double 3).  These can read either :cvector or
;;; :foreign-array.  This allows the user to define ordinary CL
;;; functions with scalars as input and output.  However, it may be
;;; desirable to read and set grid:foreign-arrays, in which case
;;; :pointer is the right specification.

;;;;****************************************************************************
;;;; Setting slots
;;;;****************************************************************************

(defun set-structure-slot (foreign-structure structure-name slot-name value)
  (setf (cffi:foreign-slot-value foreign-structure structure-name slot-name)
	value))

(defun set-slot-function (foreign-structure structure-name slot-name gsl-function)
  "Set the slot in the cbstruct to the callback corresponding to gsl-function.
   If gsl-function is nil, set to the null-pointer."
  (set-structure-slot
   foreign-structure structure-name slot-name
   (if gsl-function
       (cffi:get-callback gsl-function)
       (cffi:null-pointer))))

(defun set-parameters (foreign-structure structure-name)
  "Set the parameters slot to null."
  (set-structure-slot foreign-structure structure-name
		      'parameters (cffi:null-pointer)))

;;;;****************************************************************************
;;;; Definitions for making cbstruct
;;;;****************************************************************************

(defun set-cbstruct (cbstruct structure-name slots-values function-slotnames)
  "Make the slots in the foreign callback structure."
  (loop for (slot-name function) on function-slotnames by #'cddr
     do (set-slot-function cbstruct structure-name slot-name function))
  (set-parameters cbstruct structure-name)
  (when slots-values
    (loop for (slot-name value) on slots-values by #'cddr
       do (set-structure-slot cbstruct structure-name slot-name value))))

(defun make-cbstruct (struct slots-values &rest function-slotnames)
  "Make the callback structure."
  (assert struct (struct) "Structure must be supplied.")
  (let ((cbstruct (cffi:foreign-alloc struct)))
    (set-cbstruct cbstruct struct slots-values function-slotnames)
    cbstruct))

;;;;****************************************************************************
;;;; Parsing :callback argument specification
;;;;****************************************************************************

;;; The :callbacks (cbinfo) argument is a list of the form:
;;; (foreign-argument callback-fnstruct dimension-names function ...)
;;; where each function is of the form 
;;; (structure-slot-name
;;;   &optional (return-spec 'double-float) (argument-spec 'double-float)
;;;             set1-spec set2-spec)
;;; and each of the *-spec are (type array-type &rest dimensions).
;;; When assigned to a variable, the variable is named 'cbinfo.

(defun parse-callback-static (cbinfo component)
  "Get the information component from the callbacks list."
  (case component
    (foreign-argument (first cbinfo))
    (callback-fnstruct (second cbinfo))
    (dimension-names (third cbinfo))
    (functions (nthcdr 3 cbinfo))))

(defun number-of-callbacks (cbinfo)
  (length (parse-callback-static cbinfo 'functions)))

(defun parse-callback-fnspec (fnspec component)
  "From the :callbacks argument, parse a single function specification."
  (ecase component
    (structure-slot-name (first fnspec))
    (return-spec (second fnspec))
    (arguments-spec (cddr fnspec))))

(defun parse-callback-argspec (argspec component)
  "From the :callbacks argument, parse a single argument of a single
  function specification."
  (ecase component
    (io (first argspec))		; :input or :output
    (element-type (second argspec))	; :double
    (array-type (third argspec))	; :foreign-array or :cvector
    (dimensions (nthcdr 3 argspec))))

;;;;****************************************************************************
;;;; Using callback specification in function arugments
;;;;****************************************************************************

(defun callback-replace-arg (replacement list cbinfo)
  "Replace in the list the symbol representing the foreign callback argument."
  (if cbinfo
      (subst
       replacement
       (parse-callback-static cbinfo 'foreign-argument)
       list)
      list))

(defun callback-remove-arg (list cbinfo &optional key)
  "Remove from the list the symbol representing the foreign callback argument."
  (remove (parse-callback-static cbinfo 'foreign-argument) list :key key))

;;;;****************************************************************************
;;;; Form generation
;;;;****************************************************************************

;;; The :callback-dynamic is a list of the form
;;; (dimensions (function scalarsp) ...)
;;; This is used in defmfuns that send callbacks directly, with no mobject.
;;; With functions given in the same order as
;;; the in the :callbacks argument
;;; function = function designator, 
;;; scalarsp = flag determining whether to pass/accept scalars or arrays
;;; dimensions = source dimensions, a list

(defun cbd-dimensions (callback-dynamic)
  (first callback-dynamic))

(defun cbd-functions (callback-dynamic)
  (rest callback-dynamic))

(defun callback-symbol-set (callback-dynamic cbinfo symbols)
  "Generate the form to set each of the dynamic (special) variables
   to (function scalarsp dimensions...) in the body of the demfun for
   each of the callback functions."
  (when cbinfo
    `((setf
       ,@(loop for symb in symbols
	    for function in (cbd-functions callback-dynamic)
	    for fnspec in (parse-callback-static cbinfo 'functions)
	    append
	    `(,symb
	      (make-compiled-funcallable
	       ,(first function)
	       ',fnspec
	       ,(second function)
	       ,(cons 'list (cbd-dimensions callback-dynamic)))))))))

(defun callback-set-slots (cbinfo dynamic-variables callback-dynamic)
  "Set the slots in the foreign callback struct."
  (when cbinfo
    `((set-cbstruct
       ,(parse-callback-static cbinfo 'foreign-argument)
       ',(parse-callback-static cbinfo 'callback-fnstruct)
       ,(when (parse-callback-static cbinfo 'dimension-names)
	      (cons 'list
		    (loop
		       for dim-name in (parse-callback-static cbinfo 'dimension-names)
		       for dim in (cbd-dimensions callback-dynamic)
		       append (list `',dim-name dim))))
       ,(cons
	 'list
	 (loop for symb in (second dynamic-variables)
	    for fn in (parse-callback-static cbinfo 'functions)
	    append
	    `(',(parse-callback-fnspec fn 'structure-slot-name)
		',symb)))))))

(defun callback-args (argspec)
  "The arguments passed by GSL to the callback function."
  (loop for arg in argspec
     with count = -1
     collect
     (if (listp arg)
	 (list (make-symbol-cardinal "ARG" (incf count))
	       (if (parse-callback-argspec arg 'array-type)
		   :pointer		; C array
		   (parse-callback-argspec arg 'element-type)))
	 arg)))

;;;;****************************************************************************
;;;; Macro defmcallback
;;;;****************************************************************************

(defun make-defmcallbacks (cbinfo callback-names function-names)
  (when cbinfo
    (mapcar
     (lambda (cb vbl fspec) `(defmcallback ,cb ,vbl ,fspec))
     callback-names function-names
     (parse-callback-static cbinfo 'functions))))

(defmacro defmcallback (name dynamic-variable function-spec)
  (let* ((argspec (parse-callback-fnspec function-spec 'arguments-spec))
	 (return-type (parse-callback-fnspec function-spec 'return-spec))
	 (args (callback-args argspec))
	 (slug (make-symbol "SLUG")))
    `(cffi:defcallback ,name
	 ,(if (eq return-type :success-failure) :int return-type)
	 (,@(substitute `(,slug :pointer) :slug args))
       ;; Parameters as C argument are always ignored, because we have
       ;; CL specials to do the same job.
       (declare ,@(when (member :slug args) `((ignore ,slug)))
		(special ,dynamic-variable))
       (funcall ,dynamic-variable ,@(mapcar 'grid:st-symbol (remove :slug args))))))
