;; Functions on grids
;; Liam Healy 2010-06-19 22:25:17EDT functions.lisp
;; Time-stamp: <2014-11-27 16:57:53EST functions.lisp>

;; Copyright 2010, 2011, 2012, 2013, 2014 Liam M. Healy
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

(export '(*grid-types* gridp grid rank dimensions total-size
	  specification make-grid aref aref* contents))
(export '(make-simple-grid grid *default-grid-type*
	  *default-element-type* *default-dimensions*))

;;;;****************************************************************************
;;;; Grid type
;;;;****************************************************************************

(defparameter *grid-types* '(array)
  "A list of (disjoint) types that are accepted as grids.  Not every
  object of the given type is necessarily a grid, however.")

(defvar *additional-gridp-test* nil
  "NIL or a function of one argument, that will determine if that object is a grid.")

(defun gridp (object)
  "Object is a grid; type is returned."
  (or (find-if (lambda (type) (typep object type)) *grid-types*)
      (when *additional-gridp-test* (funcall *additional-gridp-test* object))))

(deftype grid () '(satisfies gridp))

;;;;****************************************************************************
;;;; Properties of the grid
;;;;****************************************************************************

(defgeneric rank (grid)
  (:documentation "The rank (number of dimensions) of the generalized array."))

(defgeneric dimensions (grid)
  (:documentation "A list representing the dimensions of the generalized array.")
  (:method ((object list))
    (if object
	(cons (length object)
	      (when (listp (first object))
		(dimensions (first object))))
	(list 0))))

(defgeneric total-size (grid)
  (:method ((grid t))
    (total-size-from-dimension (dimensions grid))))

(defgeneric specification (grid)
  (:documentation "The grid specification.")
  (:method ((grid t))
    (unless (gridp grid)
      (error "No specification for a non-grid ~a." grid))
    (cons
     (cons (gridp grid) (dimensions grid))
     (list (element-type grid)))))

(defgeneric affi (grid)
  (:documentation "The grid's affi."))

(defgeneric element-type (grid)
  (:documentation "The element type of the grid.")
  (:method ((grid list))
    ;; Generally lists don't have a type, so we'll go on the type of the first atom.
    ;; This may be not what we want, however; SBCL gives (type-of 1) -> BIT.
    (if (atom (first grid))
	(type-of (first grid))
	(element-type (first grid)))))

;;;;****************************************************************************
;;;; Simple grid
;;;;****************************************************************************

;;; A simple grid is one that has only one layer with a specified element type.
;;; Both 'array and 'foreign-array are simple grids.

(defparameter *default-grid-type* 'array)
(defparameter *default-element-type* 'double-float)
(defparameter *default-dimensions* '(3))

(defun make-simple-grid
    (&rest args
     &key (grid-type *default-grid-type*)
     (dimensions *default-dimensions*)
     (element-type *default-element-type*)
     (initial-element (coerce-value *default-numerical-value* element-type))
     initial-contents)
  (declare (ignorable initial-element initial-contents))
  "Make a simple grid by specfying the grid-type (default *default-grid-type*), dimensions (*default-dimensions*), element-type (*default-element-type*), and optionally initial-element or initial-contents."
  (apply
   'make-grid
   (make-specification grid-type dimensions element-type)
   (alexandria:remove-from-plist args :grid-type :dimensions :element-type)))

;;; Make simple grid from contents
(defun grid (&rest args)
  "Make the simple grid with default properties and elements as specified in args."
  (make-simple-grid :initial-contents args))

;;;;****************************************************************************
;;;; Make grid
;;;;****************************************************************************

(defun make-grid
    (specification &rest keys &key
     (initial-contents nil initial-contents-p)
     (initial-element nil)
     &allow-other-keys)
  "Make a grid object with no or literal values specified.
   The specification is of the form
   ((component-type dimensions...) ... element-type)
   If initial-contents are specified, the dimensions may be
   omitted, 
   ((component-type) ... element-type)."
  (declare (ignore initial-element))
  (when initial-contents-p (check-initial-contents initial-contents))
  (let ((spec
	 (if initial-contents
	     ;; If initial-contents are given, then its dimensions
	     ;; override what's given in the specification
	     (let ((spec (copy-tree specification)))
	       (setf (specification-dimensions spec) (dimensions initial-contents))
	       spec)
	     specification)))
    (when (find-if 'zerop (specification-dimensions spec))
      (error "Making grid with zero total dimension."))
    (apply 'make-grid-data
	   (top-spec-type spec)
	   (top-spec-dimensions spec)
	   (rest spec)
	   keys)))

(defun check-initial-contents (ic)
  (unless ic (error "Initial-contents specified as NIL.")))

(defgeneric make-grid-data
    (type dimensions rest-spec &rest keys &key &allow-other-keys)
  (:documentation
   "Make the object(s) required for storage of the data,
    if it consists of CL arrays and lists."))

;;;;****************************************************************************
;;;; Reader macro #m
;;;;****************************************************************************

;;; The reader macro #m will read a list of arguments, evaluating the
;;; contents, and construct a grid from it.  If the symbol ^
;;; occurs in the list, it indicates the object being made is a
;;; matrix, and this symbol indicates the end of each row.  It is also
;;; possible to give a list of numeric values may be given for each
;;; row also.

(defparameter *hashm-numeric-code-alist*
  '((nil . double-float)
    (1 . double-float)
    (2 . (complex double-float))
    (3 . single-float)
    (4 . (complex single-float))
    (7 . (signed-byte 8))
    (8 . (unsigned-byte 8))
    (15 . (signed-byte 16))
    (16 . (unsigned-byte 16))
    (31 . (signed-byte 32))
    (32 . (unsigned-byte 32))
    (63 . (signed-byte 64))
    (64 . (unsigned-byte 64))))

(defun hashm-eltype (n)
  "Get the appropriate element type for the numeric code n"
  (rest (assoc (if (numberp n) (mod n 100) n) *hashm-numeric-code-alist*)))

(defun hashm-numeric-code (eltype)
  "Get the appropriate element type for the numeric code n"
  (first (rassoc eltype *hashm-numeric-code-alist* :test 'equal)))

(defparameter *hashm-grid-type-alist*
  nil)

(defun hashm-grid-type (n)
  (or (and (numberp n)
	   (rest (assoc (floor n 100) *hashm-grid-type-alist*)))
      *default-grid-type*))

(defvar *row-separator* '^)
(export *row-separator*)

(defun read-grid (stream subchar arg)
   (declare (ignore subchar))
   (read-char stream)
   (let ((list (read-delimited-list #\) stream)))
     (unless *read-suppress*
       (check-initial-contents list)
       (make-grid
	;; Specification without dimensions, they will be filled in
	;; with initial-contents in make-grid.
	`((,(hashm-grid-type arg)) ,(hashm-eltype arg))
	:initial-contents
	(if (find *row-separator* list)
	    (split-sequence:split-sequence *row-separator* list)
	    list)))))

;;; Use #m to make the foreign-array
(set-dispatch-macro-character
 #\# #\m 'read-grid
 (named-readtables:find-readtable :antik))

;;;;****************************************************************************
;;;; Simple math functions
;;;;****************************************************************************

(defmethod antik:zerop ((grid array))
  (loop for ind from 0 below (total-size grid)
	always (antik:zerop (aref* grid ind))))

(defmacro sequence-map (operator scalar sequence sequence-first)
  "Apply the dyadic operator on a scalar and a sequence, applying making a similar sequence with each element to operator acting on the input element and the scalar."
  `(map (type-of ,sequence)
	(,(if sequence-first 'alexandria:rcurry 'alexandria:curry)
	 (function ,operator)
	 ,scalar)
	,sequence))

(defmacro defmethods-dyadic-sequences (operator)
  "Define defmethods for dyadic operators having at least one argument a sequence."
  `(progn
     (defmethod ,operator ((a sequence) b) ; b is a scalar
       (if (gridp b)
	   (call-next-method)
	   (sequence-map ,operator b a t)))
     (defmethod ,operator (a (b sequence)) ; a is a scalar
       (if (gridp a)
	   (call-next-method)
	   (sequence-map ,operator a b nil)))
     (defmethod ,operator ((a sequence) (b sequence)) ; both sequences
       (assert (eq (length a) (length b)) (a b)
	       "Sequence arguments are not of the same length.")
       ;; apply elementwise
       (map (type-of a) (function ,operator) a b))))

(defmethods-dyadic-sequences antik::+i)
(defmethods-dyadic-sequences antik::-i)
(defmethods-dyadic-sequences antik::*i)
(defmethods-dyadic-sequences antik::/i)

;;;;****************************************************************************
;;;; Element reference
;;;;****************************************************************************

(defgeneric aref (grid &rest indices)
  (:documentation "Select the element from the grid."))

(defgeneric (setf aref) (value grid &rest indices)
  (:documentation "Set the element from the grid."))

(defgeneric aref* (grid linearized-index)
  (:documentation "Select the element from the grid using a linearized index.")
  ;; Check the type for debugging purposes; it can be removed for production.
  (:method :before (grid linearized-index)
	   (check-type linearized-index (integer 0))))

(defgeneric (setf aref*) (value grid linearized-index)
  (:documentation "Set the element from the grid using a linearized index.")
  ;; Check the type for debugging purposes; it can be removed for production.
  (:method :before (value grid linearized-index)
	   (check-type linearized-index (integer 0))))

(defun listify (function dimensions)
  "Make the dimension list."
  (declare (type function function))
  (if dimensions
      (loop for i from 0 below (first dimensions)
	 collect (listify (alexandria:curry function i) (rest dimensions)))
      (funcall function)))

(defun contents (grid)
  "The contents of the grid as nested lists.  This can
   be used as the :initial-contents argument in making
   a new grid."
  (listify (alexandria:curry 'aref grid) (dimensions grid)))

;;;;****************************************************************************
;;;; Compatibility
;;;;****************************************************************************

;;; The predecessor system to Antik had array element access through
;;; the functions gref and gref*.  These are provided here for
;;; compatibility.

(export '(gref gref*))

(defun gref (grid &rest indices)
  "Obsolete; use grid:aref."
  (apply #'aref grid indices))

(defun gref* (grid linearized-index)
  "Obsolete; use grid:aref*."
  (aref* grid linearized-index))

(defun (setf gref) (value grid &rest indices)
  "Obsolete; use grid:aref."
  (apply #'(setf aref) value grid indices))

(defun (setf gref*) (value grid linearized-index)
  "Obsolete; use grid:aref*."
  (funcall #'(setf aref*) value grid linearized-index))
