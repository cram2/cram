;; Functions on grids
;; Liam Healy 2010-06-19 22:25:17EDT functions.lisp
;; Time-stamp: <2010-07-03 23:31:02EDT functions.lisp>

(in-package :grid)

(export '(grid-rank dimensions make-grid gref gref* contents))

;;;;****************************************************************************
;;;; Properties of the grid
;;;;****************************************************************************

(defgeneric grid-rank (grid)
  (:documentation "The rank (number of dimensions) of the generalized array."))

(defgeneric dimensions (grid)
  (:documentation "A list representing the dimensions of the generalized array.")
  (:method ((object list))
    (if object
	(cons (length object)
	      (when (listp (first object))
		(dimensions (first object))))
	(list 0))))

(defgeneric specification (grid)
  (:documentation "The grid specification.")
  (:method ((grid t))
    (cons
     (cons
      (find-if (lambda (ty) (typep grid ty)) *grid-types*)
      (dimensions grid))
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
;;;; Make grid
;;;;****************************************************************************

(defun make-grid
    (specification &rest keys &key initial-contents &allow-other-keys)
  "Make a grid object with no or literal values specified.
   The specification is of the form
   ((component-type dimensions...) ... element-type)
   If initial-contents are specified, the dimensions may be
   omitted, 
   ((component-type) ... element-type)."
  (let ((spec
	 (if (and initial-contents
		  (symbolp (caar specification)))
	     `((,(caar specification) ,@(dimensions initial-contents))
	       ,@(rest specification))
	     specification)))
    (when (find-if 'zerop (specification-dimensions spec))
      (error "Making grid with zero total dimension."))
    (apply 'make-grid-data
	   (top-spec-type spec)
	   (top-spec-dimensions spec)
	   (rest spec)
	   keys)))

(defgeneric make-grid-data
    (type dimensions rest-spec &rest keys &key &allow-other-keys)
  (:documentation
   "Make the object(s) required for storage of the data,
    if it consists of CL arrays and lists."))

;;;;****************************************************************************
;;;; Element reference
;;;;****************************************************************************

(defgeneric gref (grid &rest indices)
  (:documentation "Select the element from the grid."))

(defgeneric (setf gref) (value grid &rest indices)
  (:documentation "Set the element from the grid."))

(defgeneric gref* (grid linearized-index)
  (:documentation "Select the element from the grid using a linearized index.")
  ;; Check the type for debugging purposes; it can be removed for production.
  (:method :before (grid linearized-index)
	   (check-type linearized-index (integer 0))))

(defgeneric (setf gref*) (value grid linearized-index)
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
  (listify (alexandria:curry 'gref grid) (dimensions grid)))
