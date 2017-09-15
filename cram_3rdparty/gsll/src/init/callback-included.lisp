;; The mobject that defines callbacks
;; Liam Healy 2009-03-14 11:20:03EDT callback-included.lisp
;; Time-stamp: <2011-01-10 18:19:12EST callback-included.lisp>
;;
;; Copyright 2009, 2011 Liam M. Healy
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
;;;; Class definitions and macros
;;;;****************************************************************************

(defclass callback-included (mobject)
  ((cbinfo
    :initarg :cbinfo :reader cbinfo
    :documentation "The specification form for static callback information.")
   (dimension-names
    :initarg :dimension-names :reader dimension-names
    :documentation "The names in the GSL struct for dimensions.")
   (functions
    :initarg :functions :reader functions :initform nil
    :documentation "The user functions as function designators.
    These should correspond in order to the structure-slot-name list.")
   (funcallables
    :initarg :funcallables :reader funcallables :initform nil
    :documentation "The function objects that will be called by
    the callbacks.")
   (scalarsp
    :initarg :scalarsp :reader scalarsp :initform t
    :documentation "Whether the function expect to be passed and return
    scalars or arrays.")
   (grid:dimensions :initarg :dimensions :reader grid:dimensions))
  (:documentation
   "A mobject that includes a callback function or functions to GSL."))

(defclass callback-included-cl (callback-included)
  ((callback :initarg :callback :reader callback-struct))
  (:documentation
   "A mobject that includes a callback function or functions, in which
    the pointer to the callback structure is stored in a CL class
    slot."))

(defmacro def-ci-subclass
    (class-name superclasses documentation dimension-names)
  `(defclass ,class-name ,superclasses
     ((dimension-names :initform ',dimension-names :allocation :class))
     (:documentation ,documentation)))

(defmacro def-ci-subclass-1d
    (class-name superclasses documentation ignore)
  (declare (ignore ignore))
  `(defclass ,class-name ,superclasses
     ((dimension-names :initform nil :allocation :class)
      (grid:dimensions :initform '(1) :allocation :class)
      (scalarsp :initform T :allocation :class))
     (:documentation ,documentation)))

(defmethod print-object ((object callback-included) stream)
  (print-unreadable-object (object stream :type t :identity t)
    (when (slot-boundp object 'functions)
      (princ "for " stream)
      (princ (first (functions object)) stream)
      (princ ", " stream))
    (princ "dimensions " stream)
    (princ (grid:dimensions object) stream)))

;;;;****************************************************************************
;;;; For making mobjects
;;;;****************************************************************************

;;; This is expanded in defmfun to set the dynamic variables. 
(defun callback-set-dynamic (callback-object &optional arglist)
  "Make a form to set the dynamic variable defining callbacks."
  (when (listp callback-object)
    (setf arglist callback-object
	  callback-object (caar callback-object)))
  `((setf 
     ,@(loop for symb in
	    (let ((class (category-for-argument arglist callback-object)))
	      (mobject-fnvnames
	       class
	       (number-of-callbacks (get-callbacks-for-class class))))
	    for n from 0
	    append `(,symb (nth ,n (funcallables ,callback-object)))))))
