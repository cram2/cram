;; Make arrays from other arrays
;; Liam Healy 2009-11-11 11:33:22EST compose-array.lisp
;; Time-stamp: <2010-07-22 10:40:15EDT compose.lisp>
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

(in-package :grid)

(export '(subgrid codimension-one-subspace row column
	  transpose diagonal identity-matrix set-diagonal
	  concatenate-grids))

;;;;****************************************************************************
;;;; Drop
;;;;****************************************************************************

(defun drop (grid &key destination (drop t))
  "Remove singleton axes."
  (map-grid :source grid
	    :source-affi (affi:drop (affi grid) drop)
	    :destination destination))

;;;;****************************************************************************
;;;; Subgrid
;;;;****************************************************************************

;;; A subgrid is a grid that is somehow contained within another.  A
;;; specific kind of subgrid is a subspace, found by fixing one or
;;; more indices in the parent grid and allowing the remaining indices
;;; to cover the same range as the parent grid.

(defun subgrid (grid dimensions start &key destination drop)
  "A subgrid of the grid, e.g. a row from a matrix, or a block."
  (let* ((affi (affi:subrange (affi grid) dimensions nil start)))
    (map-grid :source grid
	      :source-affi affi
	      :destination destination
	      :destination-affi
	      (affi:drop (affi:make-affi dimensions) drop))))

(defun (setf subgrid) (subgrid grid start &optional axes)
  "Set the subgrid of the grid.  Specify the starting indices with
   start, and in the case that the subgrid has lower rank than the
   grid, which axes; default is the first (rank subgrid) axes."
  (map-grid :source subgrid
	    :destination grid
	    :destination-affi
	    (affi:drop
	     (affi:subrange
	      (affi grid)
	      (dimensions subgrid)
	      (or axes (sequential-list (grid-rank subgrid)))
	      start)
	     t)))

(defun codimension-one-subspace (grid position index &optional destination)
  "The subspace with rank one less than the argument grid."
  (subgrid
   grid
   (set-position (dimensions grid) position 1)
   (one-non-zero-list (grid-rank grid) position index)
   :destination destination
   :drop t))

(defun (setf codimension-one-subspace) (subgrid grid position index)
  (setf (subgrid
	 grid
	 (one-non-zero-list (grid-rank grid) position index)
	 (remove-position (sequential-list (grid-rank grid)) position))
	subgrid))

(defun row (grid index &optional destination)
  (codimension-one-subspace grid 0 index destination))

(defun (setf row) (subgrid grid index)
  (setf (codimension-one-subspace grid 0 index)
	subgrid))

(defun column (grid index &optional destination)
  (codimension-one-subspace grid 1 index destination))

(defun (setf column) (subgrid grid index)
  (setf (codimension-one-subspace grid 1 index)
	subgrid))

;;;;****************************************************************************
;;;; Transpose
;;;;****************************************************************************

(defun transpose (grid &key (indices '(0 1)) destination start)
  "Transpose the grid, optionally putting the result in destination at
   the element specified by the start indices."
  (let* ((affi (affi:transpose (affi grid) indices))
	 (xdims (coerce (affi:get-domain affi) 'list)))
    (map-grid :source grid
	      :source-affi affi
	      :destination destination
	      :destination-affi
	      (affi:subrange
	       (if destination
		   (affi destination)
		   (affi:make-affi xdims))
	       xdims
	       indices
	       start))))

;;;;****************************************************************************
;;;; Diagonal
;;;;****************************************************************************

(defun diagonal (grid &key (offset 0) (indices '(0 1)) destination start)
  "Select a subgrid where the two indices are equal or 
   differ by the offset, e.g. the diagonal affi for the matrix.
   The offset specifies sub- (offset<0) or super- (offset>0)
   diagonals."
  (let* ((affi (affi:diagonal (affi grid) offset indices))
	 (ddims (coerce (affi:get-domain affi) 'list)))
    (map-grid :source grid
	      :source-affi affi
	      :destination destination
	      :destination-affi
	      (affi:subrange
	       (if destination
		   (affi destination)
		   (affi:make-affi ddims))
	       ddims
	       (subseq indices 0 1)
	       start))))

(defun (setf diagonal)
    (diagonal grid &key (offset 0) (indices '(0 1)))
  "Set a subgrid where the two indices are equal or 
   differ by the offset, e.g. the diagonal affi for the matrix.
   The offset specifies sub- (offset<0) or super- (offset>0)
   diagonals.  If grid is not supplied, a grid of one higher
   dimension than diagonal with default element 0 is used."
  (map-grid :source diagonal
	    :destination grid
	    :destination-affi
	    (affi:diagonal (affi grid) offset indices))
  diagonal)

;;; This could become a more general to make a grid?
(defun make-grid-with-diagonal (diagonal &key (offset 0) initial-element)
  "Make a matrix with the default element, setting the diagonal
   to the given diagonal."
  (let ((grid
	 (make-grid
	  (merge-specification
	   (specification diagonal)
	   nil
	   (let ((arrdim
		  (+ (abs offset) (first (dimensions diagonal)))))
	     (list arrdim arrdim)))
	  :initial-element initial-element)))
    (setf (diagonal grid :offset offset) diagonal)
    grid))

(defun identity-matrix
    (dimension
     &optional (scalar 1) (type *default-grid-type*) (element-type 'double-float))
  "A scalar multiple of the identity matrix."
  (make-grid-with-diagonal
   (make-grid
    (make-specification type dimension element-type)
    :initial-element scalar)
   :initial-element 0))

(defun set-diagonal (grid function-or-value &optional (offset 0) value)
  "Set the diagonal of the grid by calling the function on the indices.
   If value is non-nil, then set it to function-or-value, ignoring
   the indices."
  (map-grid
   :source
   (if value
       (lambda (&rest args) (declare (ignore args)) function-or-value)
       function-or-value)
   :source-dims (dimensions grid)
   :destination grid
   :destination-affi (affi:diagonal (affi grid) offset '(0 1))))

;;;;****************************************************************************
;;;; Concatenate
;;;;****************************************************************************

;;; Name for this to not conflict with cl:concatenate?
(defun concatenate-grids (grid0 grid1 &key (axis 0))
  "Concatenate the grids along the axis specified.  The new grid
   will have the grid type specification and element type
   as grid0."
  (let* ((dim0 (dimensions grid0))
	 (dim1 (dimensions grid1))
	 (ri0 (remove-position dim0 axis))
	 (ri1 (remove-position dim1 axis)))
    (unless (equal ri0 ri1)
      (error "Grids cannot be concatenated due to unequal dimensions."))
    (let ((new-dims (insert-position
		     (+ (elt dim0 axis) (elt dim1 axis))
		     ri0
		     axis)))
      (let ((answer
	     (map-grid
	      :source grid0
	      :destination
	      (make-grid
	       (merge-specification (specification grid0) nil new-dims))
	      :destination-affi
	      (affi:subrange
	       (affi:make-affi new-dims)
	       (dimensions grid0)))))
	(map-grid :source grid1
		  :destination answer
		  :destination-affi
		  (affi:subrange
		   (affi answer)
		   (dimensions grid1)
		   nil
		   (let ((start (make-list (length new-dims) :initial-element 0)))
		     (setf (nth axis start) (elt dim0 axis))
		     start)))
	answer))))
