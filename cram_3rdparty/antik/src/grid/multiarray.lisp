;; Grids containing grids.
;; Liam Healy 2010-06-16 19:03:25EDT multiarray.lisp
;; Time-stamp: <2011-08-20 17:36:47EDT multiarray.lisp>
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

;;; A grid, or generalized array, is either:
;;; 1) a scalar
;;; 2) a list, array, structure, SQL data type, etc. with each element
;;; a grid, and each element identical in type and shape.
;;; The type of a grid is the set of objects (array of lists, etc.)
;;; The shape is the collected set of dimensions at each level.

;;; The sequence of content of (2) is called a "component".  Each scalar
;;; is an "element".  The number of components like (2) is the "depth".
;;; When thinking of each of (2), these are thought of geometrically so
;;; that there are axes defined.  The total number of dimensions is the
;;; rank; because arrays can themselves have more than one
;;; dimension, the depth isn't necessarily equal to the rank. 

;;; For example: an array of double-float, a list, a list of lists of
;;; equal length, an array of lists of equal length, etc.

;;; For a new kind of component of a grid, the following
;;; methods should be defined: make-grid-data, grid-ref, (setf grid-ref).

;;; Functions for use:
;;; make-grid, aref, (setf aref), element-type,reshape/gmap
;;; Internal use: linearize-grid-index, delinearize-grid-index


;;;;****************************************************************************
;;;; Grid class definition
;;;;****************************************************************************

;;; Grids as defined above include CL arrays, but the CL standard does
;;; not permit us to superclass a built-in class, so the object
;;; defined here only includes all grid objects other than pure CL
;;; arrays.

;;; For the time being scalars are excluded.

(in-package :grid)

(export '(grid grid-array))

(defclass grid ()
  ((rank :initarg :rank :type fixnum :reader rank
	 :documentation "The number of axes (dimensions).")
   (affi :initarg :affi :type affi:affi :reader affi)
   ;; Do I need total size?
   ;;(total-size :initarg :total-size :type fixnum :reader total-size)
   (specification :initarg :specification :type list :reader specification)
   (data :initarg :data :reader grid-data))
  (:documentation "A generalized array, excluding CL arrays."))

(defparameter *print-contents* t
  "Print the contents of the grid.")

(defmethod print-object ((object grid) stream)
  (print-unreadable-object (object stream :type t) 
    (if *print-contents*
	(princ (grid-data object) stream)
	(format stream "specification ~a" (specification object)))))

(defparameter *grid-data-superclasses* nil
  "A list of superclasses of possible grid classes.
   Any subclass will be made as the indicated superclass.")

(defgeneric make-grid-data
    (type dimensions rest-spec &key initial-element initial-contents)
  (:documentation
   "Make the object(s) required for storage of the data,
    if it consists of CL arrays and lists.") 
  (:method ((type (eql 'list)) dimensions rest-spec
	    &key (initial-element nil initial-element-p)
	    (initial-contents nil initial-contents-p))
    (let* ((length (first dimensions))
	   (list
	    (apply
	     'make-list
	     length
	     (when (spec-scalar-p rest-spec)
	       (list
		:initial-element
		(coerce-value initial-element (top-spec-type rest-spec)))))))
      (unless (atom (first rest-spec))
	(loop for i below length
	   with cadr-spec = (first rest-spec)
	   do (setf (nth i list)
		    (apply 
		     'make-grid-data
		     (first cadr-spec) (rest cadr-spec) (rest rest-spec)
		     (when initial-element-p
		       (list :initial-element initial-element))))))
      (when initial-contents-p (set-contents list initial-contents))
      list))
  (:method ((type (eql 'array)) dimensions rest-spec
	    &rest keys &key  (initial-element nil initial-element-p)
	    (initial-contents nil initial-contents-p))
    (let* ((element-type (top-spec-type rest-spec))
	   (array
	    (apply
	     'make-array
	     dimensions
	     :element-type element-type
	     (when (spec-scalar-p rest-spec)
	       (append
		(when initial-element-p
		  (list :initial-element (coerce-value initial-element element-type)))
		(when initial-contents-p
		  (list :initial-contents
			(mapcar (lambda (v) (coerce-value v element-type))
				initial-contents))))))))
      (when (listp (first rest-spec))
	(loop for i below (array-total-size array)
	   with cadr-spec = (first rest-spec)
	   do
	   (setf (row-major-aref array i)
		 (apply
		  'make-grid-data
		  (first cadr-spec) (rest cadr-spec) (rest rest-spec)
		  keys))))
      array)))

(defmethod make-load-form ((object grid) &optional env)
  (declare (ignore env))
  `(make-grid ',(specification object) :initial-contents ,(linear-data object)))

(defgeneric grid-ref (data &rest indices)
  (:method (data &rest indices)
    (if (null indices)
	data
	(error "Don't know how to find ~a element of ~a" indices data)))
  (:method ((data list) &rest indices)
    (apply 'grid-ref (nth (first indices) data) (rest indices)))
  (:method ((data array) &rest indices)
    (apply 'grid-ref (apply #'cl:aref data (subseq indices 0 (array-rank data)))
	   (subseq indices (array-rank data)))))

(defgeneric (setf grid-ref) (value data &rest indices)
  (:method (value (data list) &rest indices)
    (if (= 1 (length indices))
	(setf (elt data (first indices)) value)
	(apply (fdefinition '(setf grid-ref))
	       value (nth (first indices) data) (rest indices))))
  (:method (value (data array) &rest indices)
    (if (= (length indices) (array-rank data))
	;; set the value
	(setf (apply #'cl:aref data indices) value)
	;; else recurse
	(apply (fdefinition '(setf grid-ref))
	       value
	       (apply #'cl:aref data (subseq indices 0 (array-rank data)))
	       (subseq indices (array-rank data))))))



(defmethod grid:aref ((grid grid) &rest indices)
  (apply 'grid-ref (grid-data grid) indices))

(defmethod (setf grid:aref) (value (grid grid) &rest indices)
  (setf (apply #'grid-ref (grid-data grid) indices) value))

(defmethod grid:aref* ((grid grid) linearized-index)
    (apply #'grid:aref grid (affi::delinearize-index grid linearized-index)))

(defmethod (setf grid:aref*) (value (grid grid) linearized-index)
  (setf (apply #'grid:aref grid (affi::delinearize-index grid linearized-index))
	value))

;;; Needed?
(defun set-contents (object contents)
  (when contents (setf (linear-data object) contents)))

;;;;****************************************************************************
;;;; Grid maker
;;;;****************************************************************************

(defun pure-cl-array-p (specification)
  "Specification is for a pure CL array."
  (and (= (length specification) 2) (eql 'array (caar specification))))

(defun make-grid
    (specification &key
     (initial-contents nil initial-contents-p)
     (initial-element nil initial-element-p))
  "Make a grid object with no or literal values specified."
  (let* ((dimensions
	  (if (find-if 'zerop (specification-dimensions specification))
	      (error "Making grid with zero total dimension.")
	      (specification-dimensions specification)))
	 (data (apply 'make-grid-data
		      (top-spec-type specification)
		      (top-spec-dimensions specification)
		      (rest specification)
		      (append
		       (when initial-element-p
			 (list :initial-element initial-element))
		       (when initial-contents-p
			 (list :initial-contents initial-contents)))))
	 (object
	  (if (or (typep data 'grid) (pure-cl-array-p specification))
	      data	     ; A pure CL array is just returned as-is
	      (make-instance ; Everything else is put into a grid object
	       'grid
	       :rank (length dimensions)
	       :affi (affi:make-affi dimensions)
	       :specification specification
	       :data data))))
    object))

(defmethod dimensions ((grid grid))
  (coerce (affi:get-domain (affi grid)) 'list))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Obsolete definition
;;; (make-specification-gt (affi:make-affi '(3 4 5 6)) '(list list (array 2) double-float))
;;; ((LIST 3) (LIST 4) (ARRAY 5 6) DOUBLE-FLOAT)

;;; The gtype is the specification with dimensions removed, and only
;;; rank included.
;;; (list list (array 2) double-float)

;;; Merging an affi with a gtype to make a specification:
;;; Separating a specification
;;; (gtype *)
;;; (LIST LIST (ARRAY 2) DOUBLE-FLOAT)
;;; (affi:make-affi (specification-dimensions **))
;;; #<AFFI:AFFI domain #(3 4 5 6), const 0, coeff #(120 30 6 1) {1004EDF811}>
