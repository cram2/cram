;; Classes for vectors and matrices
;; Liam Healy 2010-06-26 19:10:45EDT subclass.lisp
;; Time-stamp: <2011-01-29 12:21:25EST subclass.lisp>

(in-package :grid)

(export 'data-class-name)

;;;;****************************************************************************
;;;; Definition of specific data classes
;;;;****************************************************************************

(defparameter *class-element-type* nil
  "The mapping between the class name and the CL element type.")

(defun data-class-name (category-or-rank element-type &optional making-class)
  "The class name from the type of element."
  ;; e.g. (data-class-name 'vector '(unsigned-byte 8))
  ;; -> VECTOR-UNSIGNED-BYTE-8
  (if (member category-or-rank '(vector matrix 1 2))
      (let ((class-name
	     (intern (format nil "~:@(~a-~a~)"
			     (case category-or-rank
			       ((1 vector) 'vector)
			       ((2 matrix) 'matrix))
			     (cl-single element-type :grid))
		     :grid)))
	(if (or making-class (subtypep class-name 'foreign-array))
	    class-name
	    (error "Specified element type for a foreign array ~a is not one of ~a"
		   element-type
		   (all-types *class-element-type* t))))
      (error "~a is not an acceptible category or rank." category-or-rank)))

(defun data-defclass (category superclass)
  "Define all the subclasses based on the known element types."
  (cons 'progn
	(mapcan
	 (lambda (element-type-cl)
	   (let* ((class-name (data-class-name category element-type-cl t)))
	     ;; Define the class
	     `((defclass ,class-name
		   (,superclass)
		 ((element-type :initform ',element-type-cl
				:allocation :class)))
	       ;; Push mapping onto *class-element-type*
	       (pushnew ',(cons class-name element-type-cl)
			*class-element-type* :test #'equal)
	       (export ',class-name))))
	 *array-element-types*)))
