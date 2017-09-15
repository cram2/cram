;; Cholesky Decomposition
;; Liam Healy, Wed May  3 2006 - 16:38
;; Time-stamp: <2011-05-26 12:37:33EDT cholesky.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2011 Liam M. Healy
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

;;; FDL
;;; A symmetric, positive definite square matrix A has a Cholesky
;;; decomposition into a product of a lower triangular matrix L and
;;; its transpose L^T,
;;; A = L L^T
;;; This is sometimes referred to as taking the square-root of a matrix. The
;;; Cholesky decomposition can only be carried out when all the eigenvalues
;;; of the matrix are positive.  This decomposition can be used to convert
;;; the linear system A x = b into a pair of triangular systems
;;; (L y = b, L^T x = y), which can be solved by forward and
;;; back-substitution.

;;; GSL version 1.10 introduced functions for complex matrices.

(defmfun cholesky-decomposition ((A grid:matrix))
  ("gsl_linalg" :complex "_cholesky_decomp")
  (((mpointer A) :pointer))
  :definition :generic
  :element-types :doubles
  :inputs (A)
  :outputs (A)
  :documentation			; FDL
  "Factorize the positive-definite square matrix A into the Cholesky
  decomposition A = L L^T (real) or A = L L^H (complex).  On output the
  diagonal and lower triangular part of the input matrix A contain the
  matrix L.  The upper triangular part of the input matrix contains
  L^T, the diagonal terms being identical for both L and L^T.  If the
  matrix is not positive-definite then the decomposition will fail,
  returning the error input-domain.")

(defmfun cholesky-solve
    ((A grid:matrix) (b vector) &optional x-spec
     &aux
     (x (grid:ensure-foreign-array x-spec (grid:dimensions b))))
  (("gsl_linalg" :complex "_cholesky_svx")
   ("gsl_linalg" :complex "_cholesky_solve"))
  ((((mpointer A) :pointer) ((mpointer b) :pointer))
   (((mpointer A) :pointer) ((mpointer b) :pointer)
    ((mpointer x) :pointer)))
  :definition :generic
  :element-types :doubles
  :inputs (A b)
  :outputs (x)
  :return ((or x b))
  :documentation			; FDL
  "Solve the system A x = b using the Cholesky
  decomposition of A into the matrix given by
  #'cholesky-decomposition.  If x-spec is NIL (default), the solution
  will replace b.  If x-spec is T, then an array will be created and the
  solution returned in it.  If x-spec is a grid:foreign-array, the solution will
  be returned in it.")

(defmfun cholesky-invert (cholesky)
  "gsl_linalg_cholesky_invert"
  (((mpointer cholesky) :pointer))
  :inputs (cholesky)
  :outputs (cholesky)
  :gsl-version (1 12)
  :documentation
  "Compute the inverse of the matrix cholesky which must have been
  previously computed by #'cholesky-decomposition. The inverse of the
  original matrix is stored in cholesky on output.")

;;; Examples and unit test, from linalg/test.c

(defun test-cholesky-solve-dim (matrix)
  "Solve the linear equation using Cholesky with the supplied matrix and
   a right-hand side vector which is the reciprocal of one more than
   the index."
  (cholesky-solve
   (cholesky-decomposition (copy matrix))
   (create-rhs-vector (dim0 matrix))))

(defun test-cholesky-decomp-dim (matrix)
  "Decompose using Cholesky and then multiply."
  (let ((decomp (cholesky-decomposition (copy matrix))))
    (dotimes (row (dim0 matrix) decomp)
      (loop for col from (1+ row) below (dim1 matrix) do
	   (setf (grid:aref decomp row col) 0.0d0)))
    (matrix-product decomp decomp nil 1.0d0 0.0d0 :notrans :trans)))

(defun test-cholesky-invert-dim (matrix)
  "Invert using Cholesky decomposition"
  (cholesky-invert (cholesky-decomposition (copy matrix))))

(save-test cholesky
 (test-cholesky-solve-dim *hilb2*)
 (test-cholesky-solve-dim *hilb3*)
 (test-cholesky-solve-dim *hilb4*)
 (test-cholesky-solve-dim *hilb12*)
 (test-cholesky-decomp-dim *hilb2*)
 (test-cholesky-decomp-dim *hilb3*)
 (test-cholesky-decomp-dim *hilb4*)
 (test-cholesky-decomp-dim *hilb12*)
 (test-cholesky-invert-dim *hilb2*)
 (test-cholesky-invert-dim *hilb3*)
 (test-cholesky-invert-dim *hilb4*)
 (test-cholesky-invert-dim *hilb12*))
