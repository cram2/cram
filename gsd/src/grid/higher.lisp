;; Inner product
;; Liam Healy 2009-12-13 12:23:01EST inner.lisp
;; Time-stamp: <2010-07-20 10:59:17EDT higher.lisp>
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

(in-package :grid)

;;;;****************************************************************************
;;;; Vector products
;;;;****************************************************************************

(export '(cross inner euclidean norm normalize))

(defun cross (grid0 grid1)
  "The cross product of two vectors, each of length 3."
  (make-grid
   (specification grid0)
   :initial-contents
   (list
    (- (* (gref grid0 1) (gref grid1 2))
       (* (gref grid1 1) (gref grid0 2)))
    (- (* (gref grid0 2) (gref grid1 0))
       (* (gref grid1 2) (gref grid0 0)))
    (- (* (gref grid0 0) (gref grid1 1))
       (* (gref grid1 0) (gref grid0 1))))))

(defun inner (grid0 grid1)
  "The inner product of two grids."
  (let ((w0 (affi:make-walker (affi grid0)))
	(w1 (affi:make-walker (affi grid1))))
    (loop for e0 = (funcall w0)
       for e1 = (funcall w1)
       while (and e0 e1)
       summing (* (gref* grid0 e0) (gref* grid1 e1)))))

(defun euclidean (grid &optional (kind :euclidean))
  "The norm of the grid. Kind can be :euclidean, for the 
   euclidean, or 2-norm."
  (declare (ignorable kind))
  (sqrt (inner grid grid)))

(defun norm (grid &optional (kind :euclidean))
  "The norm of the grid. Kind can be :euclidean, for the 
   euclidean norm."
  (declare (ignorable kind))
  (euclidean grid))

(defun normalize (grid)
  "Find the normalized grid, i.e., each element is divided by grid norm, and
   the normalization factor."
  (let ((norm (norm grid)))
    (values 
     (map-grid :source grid :element-function (lambda (x) (/ x norm)))
     norm)))

;;;;****************************************************************************
;;;; Make a matrix out of column vectors
;;;;****************************************************************************

(export 'matrix-from-columns)
(defun matrix-from-columns (&rest columns)
  "Make the matrix out of the equal-length vectors.
   If *default-grid-type* is non-nil, resultant grid
   will be of the specified type; otherwise, it will
   be the same as the first column."
  (assert 
   (apply '=
	  (mapcar
	   (lambda (c) (when (= 1 (length (dimensions c))) (dim0 c)))
	   columns))
   (columns)
   "Columns must be vectors of equal length.")
  (let ((ans (make-grid
	      (make-specification
	       (or *default-grid-type* (gridp (first columns)))
	       (list (dim0 (first columns)) (length columns))
	       (element-type (first columns))))))
    (loop for j from 0
       for col in columns
       do (setf (column ans j) col))
    ans))
