;; A grid:foreign-array with added metadata for GSL.
;; Liam Healy 2008-04-06 21:23:41EDT
;; Time-stamp: <2016-06-14 23:30:39EDT foreign-array.lisp>
;;
;; Copyright 2008, 2009, 2010, 2011, 2016 Liam M. Healy
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
;;;; Make the mpointer, block-pointer, size metadata
;;;;****************************************************************************

;;; We don't allocate or free the C array data, because that is
;;; handled by grid:foreign-array.  We can use the GSL functions
;;; *_alloc_from_block because they will allocate only the structure,
;;; but we must use CFFI to allocate the block structure; otherwise
;;; GSL would try to allocate the C array.  We do not use any of the
;;; GSL free functions because they would free the C array data.

(defun make-gsl-metadata (object)
  "Make the necessary GSL metadata (mpointer and block-pointer)
   for the given foreign array, and return the mpointer.
   This should only be called by #'mpointer the first time
   it is called on a particular foreign-array."
  ;; Don't do anything if mpointer has already been assigned.
  (unless (grid:metadata-slot object 'mpointer)
    (when (zerop (size object))
      (error "Object ~a has zero total dimension." object))
    (let ((blockptr (cffi:foreign-alloc '(:struct gsl-block-c))))
      (setf (cffi:foreign-slot-value blockptr '(:struct gsl-block-c) 'size)
	    (size object)
	    (cffi:foreign-slot-value blockptr '(:struct gsl-block-c) 'data)
	    (grid:foreign-pointer object)
	    (grid:metadata-slot object 'block-pointer) blockptr)
      (let ((array-struct (alloc-from-block object blockptr)))
	(setf
	 (grid:metadata-slot object 'mpointer) array-struct
	 ;; alloc-from-block automatically copies over the data pointer
	 ;; from the block to the vector/matrix; we must do that manually here
	 (cffi:foreign-slot-value
	  array-struct
	  (if (typep object 'grid:matrix)
	      '(:struct gsl-matrix-c)
	      '(:struct gsl-vector-c))
	  'data)
	 (grid:foreign-pointer object))
	(tg:finalize object
		     (lambda ()
		       (cffi:foreign-free blockptr)
		       (cffi:foreign-free array-struct))))
      (grid:metadata-slot object 'mpointer))))

(defmethod mpointer ((object grid:foreign-array))
  (or 
   (grid:metadata-slot object 'mpointer)
   (make-gsl-metadata object)))

;;;;****************************************************************************
;;;; Make from GSL mpointers 
;;;;****************************************************************************

;; Some functions in solve-minimize-fit return a pointer to a GSL
;; vector of double-floats.  This function turns it into a
;; foreign-array.  

(defun make-foreign-array-from-mpointer
    (mpointer
     &optional (element-type 'double-float) (category-or-rank 'vector) finalize)
  "Make the foreign array when a GSL pointer to a gsl-vector-c or gsl-matrix-c is given."
  (let* ((category
	  (case category-or-rank
	    ((vector :vector 1) 'vector)
	    ((matrix grid:matrix :matrix 2) 'grid:matrix)
	    (t (error "Unrecognized category ~a" category-or-rank))))
	 (cstruct
	  (case category
	    (vector '(:struct gsl-vector-c))
	    (grid:matrix '(:struct gsl-matrix-c))))
	 (fa
	  (grid:make-grid
	   (case category
	     (vector
		`((grid:foreign-array ,(cffi:foreign-slot-value mpointer cstruct 'size))
		  ,element-type))
	     (grid:matrix
		`((grid:foreign-array
		   ,(cffi:foreign-slot-value mpointer cstruct 'size0)
		   ,(cffi:foreign-slot-value mpointer cstruct 'size1))
		  ,element-type)))
	   :foreign-pointer
	   (cffi:foreign-slot-value mpointer cstruct 'data)
	   :finalizer finalize)))
    (setf (grid:metadata-slot fa 'mpointer) mpointer)
    fa))
