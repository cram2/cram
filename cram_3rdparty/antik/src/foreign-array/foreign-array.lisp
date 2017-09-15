;; Foreign (C or C-compatible) arrays
;; Liam Healy 2008-12-28 10:44:22EST foreign-array.lisp
;; Time-stamp: <2013-11-30 15:34:43EST foreign-array.lisp>
;;
;; Copyright 2008, 2009, 2010, 2011, 2012, 2013 Liam M. Healy
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

(in-package :grid)

;;; A foreign-array is an array in foreign memory.  If the CL
;;; implementation supports static-vectors and it is single
;;; dimensional or linearization is :row-major, the same array will
;;; appear as a CL array in the slot 'cl-array.

;;;;****************************************************************************
;;;; Object and methods
;;;;****************************************************************************

(export '(foreign-array foreign-pointer dimensions 
	  cl-array metadata-slot *print-contents*
	  make-foreign-array
	  make-fortran-array
	  ensure-foreign-array
	  make-foreign-array-from-pointer))

(eval-when (:compile-toplevel :load-toplevel :execute)
  (pushnew 'foreign-array *grid-types*))

(defclass foreign-array ()
  ((foreign-pointer :initarg :foreign-pointer :reader foreign-pointer)
   (dimensions :initarg :dimensions :reader dimensions)
   ;;(rank :initarg :rank :reader rank)
   (total-size :reader total-size)
   (element-type :initarg :element-type :reader element-type
		 :initform *default-element-type*)
   (affi :reader affi)
   (cl-array :reader cl-array :initform nil)
   (foreign-metadata
    :reader foreign-metadata :initarg :foreign-metadata :initform nil))
  (:documentation
   "Arrays that can be referenced by foreign functions."))

;;; Foreign complex arrays at present are only supported on
;;; static-vectors.  Work needs to be done to set elements in the case
;;; of non-static-vectors [[id:f6b82bef-3dd2-4371-aa0e-eda4c9b8b781]].

(defmethod initialize-instance :after
    ((object foreign-array)
     &key dimensions element-type foreign-pointer finalizer
     (linearization :row-major)		; :row-major or :column-major
     initial-element initial-contents)
  ;; For multidimensional arrays, initial-contents can be flat (unlike
  ;; make-array) or with the proper depth (as for make-array).
  (assert (member element-type *array-element-types* :test 'equal)
	  (element-type)
	  "Element type ~a not permitted." element-type)
  (assert (or (null dimensions) (and (listp dimensions) (every 'integerp dimensions)))
	  (dimensions)
	  "Dimensions must be a list of positive integers or nil; ~a is not acceptable." dimensions)
  (setf	(slot-value object 'total-size)
	(total-size-from-dimension dimensions)
	(slot-value object 'affi)
	(case linearization
	  (:column-major (affi:make-affi-cm dimensions))
	  (t (affi:make-affi dimensions)))
	;;(slot-value object 'rank) (length dimensions)
	)
  (unless foreign-pointer
    (let* ((ic (coerce-value (alexandria:flatten initial-contents) element-type))
	   (ict (if (eq linearization :column-major)
		    ;; transpose initial contents
		    (loop with map
			 = (affi::map-affi (affi:make-affi dimensions)
					   (affi object))
			 for i from 0 below (length ic)
			 collect (nth (funcall map i) ic))
		    ic))
	   (ivc (when initial-element
		  (coerce-value initial-element element-type))))
      (if (and (member :static-vectors *features*)
	       ;; Some implementations do not support complex element types as static vectors
	       #+(or ccl lispworks) (not (subtypep element-type 'complex))
	       (or (= 1 (length dimensions))
		   (eql linearization :row-major)))
	  ;; Make a CL image array if the implementation supports it and
	  ;; this is a vector (one dimensional) or multi-dimensional
	  ;; array in row-major order (e.g., C, not Fortran).
	  #+static-vectors
	  (let ((sv
		 (apply
		  'static-vectors:make-static-vector
		  (total-size object)
		  :element-type element-type
		  (append
		   (when initial-element (list :initial-element ivc))
		   (when initial-contents (list :initial-contents ict))))))
	    (tg:finalize object (lambda () (static-vectors:free-static-vector sv)))
	    (setf (slot-value object 'cl-array)
		  (if (= 1 (length dimensions))
		      sv
		      (make-array dimensions
				  :element-type element-type
				  :displaced-to sv))
		  (slot-value object 'foreign-pointer)
		  (static-vectors:static-vector-pointer sv)))
	  #-static-vectors nil
	  ;; Otherwise, just set up the foreign array.
	  (let ((fv
		 (apply
		  'cffi:foreign-alloc
		  (cl-cffi element-type)
		  :count (total-size object)
		  (append
		   (when (and initial-element (not (subtypep element-type 'complex)))
		     (list :initial-element ivc))
		   (when (and initial-contents (not (subtypep element-type 'complex)))
		     (list :initial-contents ict))))))
	    (setf (slot-value object 'foreign-pointer) fv)
	    ;; Set the complex initial-contents separately
	    (when (subtypep element-type 'complex)
	      (when initial-element
		(loop for index from 0 below (total-size object)
		   do (setf (aref* object index) ivc)))
	      (when initial-contents
		(loop for elt in ict 
		   for index from 0 below (total-size object)
		   do (setf (aref* object index) elt))))
	    (tg:finalize object (lambda () (cffi:foreign-free fv)))))))
  ;; If the foreign-pointer was supplied, a finalizer will not be
  ;; attached unless requested; the assumption is that the foreign
  ;; code will take care of freeing memory.
  (when (and foreign-pointer finalizer)	; a finalizer was requested
    (tg:finalize object (lambda () (cffi:foreign-free foreign-pointer)))))

(defparameter *print-contents* t
  "Print the contents of the foreign-array.")

(defmethod print-grid-readably ((object foreign-array) contents-writer stream)
  (format stream "#~@[~d~]m" (hashm-numeric-code (element-type object)))
  (funcall contents-writer stream))

(defmethod print-object ((object foreign-array) stream)
  (print-unreadable-object (object stream :type t) 
    (if *print-contents*
	(if (slot-boundp object 'foreign-pointer)
	    (princ (contents object) stream)
	    (princ "with no pointer" stream))
	(format stream "dimensions ~a" (dimensions object)))))

(defmacro metadata-slot (object name)
  "Access a slot in the foreign-metadata."
  `(getf (slot-value ,object 'foreign-metadata) ,name))

(defmethod make-load-form ((object foreign-array) &optional env)
  (declare (ignore env))
  (check-initial-contents (contents object))
  `(make-grid
    '((foreign-array ,@(dimensions object)) ,(element-type object))
    :initial-contents ',(contents object)))

(push '(1 . foreign-array) *hashm-grid-type-alist*)

;;;;****************************************************************************
;;;; Make foreign-arrays
;;;;****************************************************************************

(defun make-foreign-array (element-type &rest keys &key dimensions &allow-other-keys)
  "Make a foreign array with the given element type,
   :dimensions, and :initial-contents, :initial-element or :data."
  (when (subtypep element-type 'foreign-array)
    (error "Can't take a class name here anymore, sorry."))
  (apply
   'make-grid
   (make-specification 'foreign-array dimensions element-type)
   keys))

(defun make-fortran-array (element-type &rest keys)
  "Make an array in column-major order with the given element type,
   :dimensions, and :initial-contents, :initial-element or :data."
  (apply 'make-instance
	 'foreign-array
	 :element-type element-type
	 :linearization :column-major
	 :finalizer t			; if we're making it, we should have a finalizer
	 keys))

(defun ensure-foreign-array
    (object
	&optional
	(dimensions *default-dimensions*)
      (element-type *default-element-type*)
      initial-element)
  "If object is not a foreign array, make and return a foreign array
   with the dimensions and element-type.  If object is a
   foreign array, it is returned."
  (if (typep object 'foreign-array)
      object
      (if initial-element
	  (make-foreign-array
	   element-type :dimensions dimensions :initial-element initial-element)
	  (make-foreign-array element-type :dimensions dimensions))))

(defun make-foreign-array-from-pointer
    (pointer dimensions element-type &optional finalize)
  "Given a foreign pointer to an array, make a Lisp object of 
   class 'foreign-array that references that.  This will never
   make a static-vector.  If finalize is true, than the array
   memory will be freed when this object is garbage collected;
   otherwise it is presumed that the foreign code will handle
   that job."
  (make-instance
   (data-class-name (length dimensions) element-type)
   :foreign-pointer pointer
   :dimensions dimensions
   :element-type element-type
   :finalizer finalize))

#|
;;; Establish a restart that takes cl-array if the calling form is
;;; looking for an array and gets a foreign-array.
(defmacro use-cl-array (&body body)
  "Use the cl-array on foreign-arrays."
  `(handler-bind
       ((type-error
	 (lambda (c)
	   (let ((obj (type-error-datum c)))
	     (when (and (typep obj 'foreign-array)
			(subtypep (type-error-expected-type c) 'array)
			(cl-array obj))
	       (use-value (cl-array obj)))))))
     ,@body))
|#

#|
(defparameter *test-fv* 
  (make-instance 'foreign-array :dimensions '(3)
		 :element-type 'double-float
		 :initial-contents '(1.0d0 2.0d0 3.0d0)))

(make-grid-data 'foreign-array '(2 2) '(double-float) :initial-element pi)
|#
