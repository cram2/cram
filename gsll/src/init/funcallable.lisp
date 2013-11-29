;; Generate a lambda that calls the user function; will be called by callback.
;; Liam Healy 
;; Time-stamp: <2010-07-13 21:59:01EDT funcallable.lisp>
;;
;; Copyright 2009, 2010 Liam M. Healy
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
;;;; Utility 
;;;;****************************************************************************

(defun value-from-dimensions (argspec dimension-values &optional total)
  "Return a list of numerical sizes for dimensions of an array.  If
   total = T, then return the product of those dimensions."
  ;; (DIM0) -> numerical value
  ;; (DIM0 DIM0) -> product of numerical values
  (let ((list
	 (subst (first dimension-values)
		'dim0
		(subst (second dimension-values)
		       'dim1
		       (parse-callback-argspec argspec 'dimensions)))))
    (if total
	(apply '* list) list)))

(defun all-io (direction &optional (arrays t))
  "Create a function that returns dimensions for argspecs that are arrays
   for the specified direction."
  (if arrays
      (lambda (arg)
	(and (listp arg)
	     (eql (parse-callback-argspec arg 'io) direction)
	     (parse-callback-argspec arg 'dimensions)))
      (lambda (arg)
	(and (listp arg)
	     (eql (parse-callback-argspec arg 'io) direction)))))

(defun vspecs-direction (argspecs direction &optional array-only)
  "Find the specs for all variables, or all array variables, with the
   specified direction."
  (remove-if-not
   (lambda (arg)
     (when
	 (and (eql (parse-callback-argspec arg 'io) direction)
	      (if array-only (parse-callback-argspec arg 'dimensions) t))
       arg))
   argspecs))

(defun faify-form (ptr argspec)
  "Make the form that turns the mpointer into a foreign-array."
  ;; No finalizer, because pointer might be reused by GSL.
  (ecase (parse-callback-argspec argspec 'array-type)
    (:foreign-array			; a GSL mpointer
     `(make-foreign-array-from-mpointer
       ,ptr
       ',(grid:cffi-cl (parse-callback-argspec argspec 'element-type))
       ,(length (parse-callback-argspec argspec 'dimensions))
       nil))				; no finalizer
    (:cvector				; a raw C vector
     `(grid:make-foreign-array-from-pointer
       ,ptr
       ',(parse-callback-argspec argspec 'dimensions)
       ',(grid:cffi-cl (parse-callback-argspec argspec 'element-type))
       nil))
    ((nil) ptr)))

;;;;****************************************************************************
;;;; Reference foreign elements and make multiple-value-bind form
;;;;****************************************************************************

(defun reference-foreign-element
    (foreign-variable-name linear-index argspec dimension-values)
  "Form to reference, for getting or setting, the element of a foreign
   array, or a scalar."
  (if (parse-callback-argspec argspec 'dimensions)
      (if (eql (parse-callback-argspec argspec 'array-type) :foreign-array)
	  `(get-value
	    ',(grid:data-class-name
	       (length (value-from-dimensions argspec dimension-values))
	       (grid:cffi-cl (parse-callback-argspec argspec 'element-type)))
	    ,foreign-variable-name
	    ,@(affi::delinearize-index
	       (affi:make-affi (value-from-dimensions argspec dimension-values))
	       linear-index))
	  `(cffi:mem-aref
	    ,foreign-variable-name
	    ',(parse-callback-argspec argspec 'element-type)
	    ,linear-index))
      ;; not setfable if it's a scalar
      foreign-variable-name))

(defun array-element-refs (names argspecs dimension-values)
  "A list of forms reference each array element in succession.
   If there is no argspec for the argument, just reference the variable itself."
  (if argspecs
      (loop for arg in argspecs
	 for count = (when arg (value-from-dimensions arg dimension-values t))
	 for name in names
	 append
	 (if arg
	     (loop for i from 0 below count
		collect (reference-foreign-element name i arg dimension-values))
	     (list name)))
      names))

(defun callback-set-mvb (argument-names form fnspec dimension-values)
  "Create the multiple-value-bind form in the callback to set the return C arrays."
  (let* ((setargs		 ; arguments that are arrays being set
	  (remove-if-not
	   (all-io :output)
	   (parse-callback-fnspec fnspec 'arguments-spec)))
	 (counts		  ; number of scalars for each set arg
	  (mapcar (lambda (arg) (value-from-dimensions arg dimension-values t))
		  setargs))
	 (count				; total number of scalars set
	  (apply '+ counts))
	 (mvbvbls	      ; the symbols to be multiple-value-bound
	  (loop for i from 0 below count
	     collect (make-symbol-cardinal 'setscalar i)))
	 (setvbls (array-element-refs argument-names setargs dimension-values)))
    (if (zerop count)
	form
	`(multiple-value-bind ,mvbvbls
	     ,form
	   (setf ,@(loop for count in counts
		      with svs = (copy-list setvbls)
		      and mvv = (copy-list mvbvbls)
		      append
		      (loop for i from 0 below count
			 append (list (pop svs) (pop mvv)))))))))


;;;;****************************************************************************
;;;; Create a lambda form suitable for call by defmcallback
;;;;****************************************************************************

(defun make-funcallable-form (user-function fnspec scalarsp dimension-values)
  "Define a wrapper function to interface GSL with the user's function.
   scalarsp will be either T or NIL, depending on whether the user function
   expects and returns scalars, and dimension-values should be a list
   of number(s), (dim0) or (dim0 dim1), or NIL."
  (let* ((argspecs (remove :slug (parse-callback-fnspec fnspec 'arguments-spec)))
	 (inargs-specs (vspecs-direction argspecs :input))
	 (inargs-names
	  (make-symbol-cardinals
	   'input
	   (length (remove nil (mapcar (all-io :input nil) argspecs)))))
	 (outarrayp (vspecs-direction argspecs :output t))
	 (outargs-names (make-symbol-cardinals 'output (length outarrayp)))
	 (lambda-args
	  (loop for arg in argspecs
	     with oarg = (copy-list outargs-names)
	     and iarg = (copy-list inargs-names)
	     append
	     (if (and (eql (parse-callback-argspec arg 'io) :output)
		      (parse-callback-argspec arg 'array-type))
		 (list (pop oarg))
		 (if (eql (parse-callback-argspec arg 'io) :input)
		     (list (pop iarg))))))
	 (function-designator
	  (if (symbolp user-function)
	      (let ((uf user-function)) `',uf)
	      user-function)))
    `(lambda ,lambda-args
       ,(if (and scalarsp (or inargs-specs outarrayp))
	    (let ((call-form
		   `(funcall
		     ,function-designator
		     ,@(array-element-refs inargs-names inargs-specs dimension-values))))
	      (if outarrayp
		  (callback-set-mvb outargs-names call-form fnspec dimension-values)
		  ;; no specified output, return what the function returns
		  call-form))
	    `(funcall ,function-designator
		      ,@(if outarrayp
			    (append (mapcar 'faify-form inargs-names inargs-specs)
				    (mapcar 'faify-form outargs-names outarrayp))
			    ;; no arrays to return, just return the value
			    (mapcar 'faify-form inargs-names inargs-specs))))
       ,@(case
	  (parse-callback-fnspec fnspec 'return-spec)
	  (:success-failure
	   ;; We always return success, because if there is a
	   ;; problem, a CL error should be signalled.
	   '(+success+))
	  (:pointer
	   ;; For unclear reasons, some GSL functions want callbacks
	   ;; to return a void pointer which is apparently meaningless.
	   '((cffi:null-pointer)))
	  ;; If it isn't either of these things, return what the
	  ;; function returned.
	  (otherwise nil)))))

(defun make-funcallables-for-object (object)
  "Make compiled functions for the object that can be funcalled in the callback."
  (setf
   (slot-value object 'funcallables)
   (mapcar
    (lambda (fn fnspec)
      (compile
       nil
       (make-funcallable-form fn fnspec (scalarsp object) (dimensions object))))
    (functions object)
    (parse-callback-static (cbinfo object) 'functions))))

(defun make-compiled-funcallable (function fnspec scalarsp dimensions)
  (compile nil (make-funcallable-form function fnspec scalarsp dimensions)))
