;; Matrices
;; Liam Healy 2008-04-15 21:57:52EDT matrix.lisp
;; Time-stamp: <2012-01-13 12:01:35EST matrix.lisp>
;;
;; Copyright 2008, 2009, 2011 Liam M. Healy
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

;;; /usr/include/gsl/gsl_matrix_double.h

;;;;****************************************************************************
;;;; Mathematical
;;;;****************************************************************************

(defmfun set-identity ((matrix grid:matrix))
  ("gsl_matrix" :type "_set_identity")
  (((mpointer matrix) :pointer))
  :definition :generic
  :c-return :void
  :outputs (matrix)
  :return (matrix)
  :documentation			; FDL
  "Set the elements of the matrix to the
  corresponding elements of the identity matrix, m(i,j) =
  \delta(i,j), i.e. a unit diagonal with all off-diagonal elements zero.
  This applies to both square and rectangular matrices.")

;;;;****************************************************************************
;;;; Copying rows and columns
;;;;****************************************************************************

(defmfun row ((matrix grid:matrix) i
     &optional (vector (grid:make-foreign-array element-type :dimensions (dim1 matrix))))
  ("gsl_matrix" :type "_get_row")
  (((mpointer vector) :pointer) ((mpointer matrix) :pointer) (i :sizet))
  :definition :generic
  :inputs (matrix)
  :outputs (vector)
  :return (vector)
  :documentation			; FDL
  "Copy the elements of the ith row of the matrix
   into the vector.  The length of the vector must be the
   same as the length of the row.")

(defmfun (setf row) ((vector vector) (matrix grid:matrix) i)
  ("gsl_matrix" :type "_set_row")
  (((mpointer matrix) :pointer) (i :sizet) ((mpointer vector) :pointer))
  :definition :generic
  :inputs (vector matrix)
  :outputs (matrix)
  :return (vector)			;setf should return the quantity set
  :documentation			; FDL
  "Copy the elements of the vector into the jth row of the matrix.
  The length of the vector must be the same as the length of the row.")

(defmfun column ((matrix grid:matrix) i
     &optional (vector (grid:make-foreign-array element-type :dimensions (dim0 matrix))))
  ("gsl_matrix" :type "_get_col")
  (((mpointer vector) :pointer) ((mpointer matrix) :pointer) (i :sizet))
  :definition :generic
  :inputs (matrix)
  :outputs (vector)
  :return (vector)
  :documentation			; FDL
  "Copy the elements of the ith column of the matrix
   into the vector.  The length of the vector must be the
   same as the length of the column.")

(defmfun (setf column) ((vector vector) (matrix grid:matrix) i)
  ("gsl_matrix" :type "_set_col")
  (((mpointer matrix) :pointer) (i :sizet) ((mpointer vector) :pointer))
  :definition :generic
  :inputs (vector matrix)
  :outputs (matrix)
  :return (vector)			;setf should return the quantity set
  :documentation			; FDL
  "Copy the elements of the vector into the ith column of the matrix.
  The length of the vector must be the same as the length of the column.")

;;;;****************************************************************************
;;;; Exchanging rows and columns
;;;;****************************************************************************

(defmfun swap-rows ((matrix grid:matrix) i j)
  ("gsl_matrix" :type "_swap_rows")
  (((mpointer matrix) :pointer) (i :sizet) (j :sizet))
  :definition :generic
  :return (matrix)
  :inputs (matrix)
  :outputs (matrix)
  :documentation 			; FDL
  "Exchange the ith and jth rows of the matrix in-place.")

(defmfun swap-columns ((matrix grid:matrix) i j)
  ("gsl_matrix" :type "_swap_columns")
  (((mpointer matrix) :pointer) (i :sizet) (j :sizet))
  :definition :generic
  :return (matrix)
  :inputs (matrix)
  :outputs (matrix)
  :documentation 			; FDL
  "Exchange the ith and jth columns of the matrix in-place.")

(defmfun swap-row-column ((matrix grid:matrix) i j)
  ("gsl_matrix" :type "_swap_rowcol")
  (((mpointer matrix) :pointer) (i :sizet) (j :sizet))
  :definition :generic
  :return (matrix)
  :inputs (matrix)
  :outputs (matrix)
  :documentation 			; FDL
  "Exchange the ith row and jth column of the
   matrix in-place.  The matrix must be square for this operation to
   be possible.")

(defmfun matrix-transpose* ((matrix grid:matrix))
  ("gsl_matrix" :type "_transpose")
  (((mpointer matrix) :pointer))
  :definition :generic
  :return (matrix)
  :inputs (matrix)
  :outputs (matrix)
  :documentation 			; FDL
  "Replace the matrix by its transpose by copying the elements
   of the matrix in-place.  The matrix must be square for this
   operation to be possible.")

(defmfun matrix-transpose
    ((source grid:matrix)
     &optional
     (destination
      (grid:make-foreign-array element-type :dimensions (reverse (grid:dimensions source)))))
  ("gsl_matrix" :type "_transpose_memcpy")
  (((mpointer destination) :pointer) ((mpointer source) :pointer))
  :definition :generic
  :return (destination)
  :inputs (source)
  :outputs (destination)
  :documentation 			; FDL
  "Make the destination matrix the transpose of the source matrix
   by copying the elements.  The dimensions of the destination
   matrix must match the transposed dimensions of the source.")
