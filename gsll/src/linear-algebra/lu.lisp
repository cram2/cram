;; LU decomposition
;; Liam Healy, Thu Apr 27 2006 - 12:42
;; Time-stamp: <2010-07-07 14:24:59EDT lu.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009 Liam M. Healy
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

;;; /usr/include/gsl/gsl_linalg.h

(defmfun LU-decomposition
    ((A matrix) &optional (permutation (make-permutation (dim0 A))))
  ("gsl_linalg" :complex "_LU_decomp")
  (((mpointer A) :pointer) ((mpointer permutation) :pointer)
   (signum (:pointer :int)))
  :definition :generic
  :inputs (A)
  :outputs (A permutation)
  :return (A permutation signum)
  :element-types :doubles
  :documentation			; FDL
  "Factorize the square matrix A into the LU decomposition PA = LU,
  and return the sign of the permutation.  On output the diagonal and
  upper triangular part of the input matrix A contain the matrix U.
  The lower triangular part of the input matrix (excluding the
  diagonal) contains L.  The diagonal elements of L are unity, and are
  not stored.

  The permutation matrix P is encoded in the permutation supplied as
  the second argument and returned as the second value.  The j-th
  column of the matrix P is given by the k-th column of the identity
  matrix, where k = p_j the j-th element of the permutation
  vector. The sign of the permutation is returned as the second value;
  it is the value (-1)^n, where n is the number of interchanges in the
  permutation.

  The algorithm used in the decomposition is Gaussian Elimination with
  partial pivoting (Golub & Van Loan, Matrix Computations,
  Algorithm 3.4.1).")

(defmfun LU-solve
    ((A matrix) (b vector) permutation &optional x-spec
     &aux
     (x (if (eq x-spec t)
	    (grid:make-foreign-array element-type :dimensions (dimensions b))
	    x-spec)))
  (("gsl_linalg" :complex "_LU_svx")
   ("gsl_linalg" :complex "_LU_solve"))
  ((((mpointer A) :pointer)
    ((mpointer permutation) :pointer) ((mpointer b) :pointer))
   (((mpointer A) :pointer)
    ((mpointer permutation) :pointer) ((mpointer b) :pointer)
    ((mpointer x) :pointer)))
  :definition :generic
  :element-types :doubles
  :inputs (A b permutation)
  :outputs (x b)		  ; depends on switch; both to be sure
  :return ((or x b))
  :documentation			; FDL
  "Solve the square system A x = b using the LU
  decomposition of A into (LU, p) given by LU-decomp.
  If x-spec is nil, the solution will be computed in-place replacing b,
  if it is T, an appropriate vector will be created and the solution
  will be computed there.  Otherwise it should be a supplied vector.")

(defmfun LU-refine
    ((A matrix) LU p (b vector) (x vector)
     &optional (residual (grid:make-foreign-array element-type :dimensions (dim0 A))))
  ("gsl_linalg" :complex "_LU_refine")
  (((mpointer A) :pointer) ((mpointer LU) :pointer)
   ((mpointer p) :pointer)
   ((mpointer b) :pointer) ((mpointer x) :pointer)
   ((mpointer residual) :pointer))
  :definition :generic
  :inputs (LU p)
  :outputs (x residual)
  :return (x)
  :element-types :doubles
  :documentation			; FDL
  "Apply an iterative improvement to x, the solution of
  A x = b, using the LU decomposition of A into (LU,p). The initial
  residual r = A x - b is also computed and stored in residual. ")

(defmfun LU-invert ((LU matrix) p inverse)
  ("gsl_linalg" :complex "_LU_invert")
  (((mpointer LU) :pointer) ((mpointer p) :pointer)
   ((mpointer inverse) :pointer))
  :definition :generic
  :inputs (LU p)
  :outputs (inverse)
  :element-types :doubles
  :documentation			; FDL
  "Compute the inverse of a matrix A from its LU
   decomposition (LU,p), storing the result in the matrix inverse. The
   inverse is computed by solving the system A x = b for each column of
   the identity matrix. It is preferable to avoid direct use of the
   inverse whenever possible, as the linear solver functions can obtain
   the same result more efficiently and reliably (consult any
   introductory textbook on numerical linear algebra for details).")

 (defmfun LU-determinant ((LU matrix) signum)
  ("gsl_linalg" :complex "_LU_det")
  (((mpointer LU) :pointer) (signum :int))
  :definition :generic
  :inputs (LU)
  :element-types :doubles
  :c-return :double
  :documentation			; FDL
  "Compute the determinant of a matrix from its LU
  decomposition, LU. The determinant is computed as the product of the
  diagonal elements of U and the sign of the row permutation signum.")

(defmfun LU-log-determinant ((LU matrix))
  ("gsl_linalg" :complex "_LU_lndet")
  (((mpointer LU) :pointer))
  :definition :generic
  :inputs (LU)
  :element-types :doubles
  :c-return :double
  :documentation			; FDL
  "The logarithm of the absolute value of the
   determinant of a matrix A, ln|det(A)|, from its LU decomposition,
   LU. This function may be useful if the direct computation of the
   determinant would overflow or underflow.")

(defmfun LU-sgndet ((LU matrix) signum)
  ("gsl_linalg" :complex "_LU_sgndet")
  (((mpointer LU) :pointer) (signum :int))
  :definition :generic
  :inputs (LU)
  :element-types :doubles
  :c-return :int
  :documentation 			; FDL
  "Compute the sign or phase factor of the determinant of a matrix A,
  det(A)/|det(A)|, from its LU decomposition, LU.")

;;; Invert a matrix using LU
(export 'invert-matrix)
(defun invert-matrix (mat)
  "Invert the matrix."
  (let* ((dim (first (dimensions mat)))
	 (per (make-permutation dim))
	 (inv (grid:make-foreign-array 'double-float :dimensions (list dim dim))))
    (LU-decomposition mat per)
    (lu-invert mat per inv)))

;;; Examples and unit test

;;; These are direct tests of matrix inversion

(save-test
 lu
 (grid:copy-to
  (invert-matrix
   (grid:make-foreign-array 'double-float
		:dimensions  '(2 2)
		:initial-contents '(1.0d0 2.0d0 3.0d0 4.0d0)))))

(generate-all-array-tests lu :doubles
 (let ((matrix (array-default '(4 4)))
       (vec (array-default '4 )))
   (multiple-value-bind (matrix perm)
       (lu-decomposition matrix)
    (let ((x (lu-solve matrix vec perm)))
      (grid:copy-to
       (permute-inverse
	perm
	(matrix-product-triangular
	 matrix
	 (matrix-product-triangular matrix x 1 :upper :notrans :nonunit)
	 1 :lower :notrans :unit)))))))

;;; From linalg/test.c

(defun test-lu-solve-dim (matrix &optional vector)
  "Solve the linear equation using LU with the supplied matrix and
   a right-hand side vector which is the reciprocal of one more than
   the index."
  (let* ((dim (dim0 matrix))
	 (rhs (or vector
		  (create-rhs-vector dim (element-type matrix)))))
    (multiple-value-bind (upper permutation signum)
	(LU-decomposition (copy matrix))
      (declare (ignore signum))
      (let ((initial-solution (LU-solve upper rhs permutation T)))
	(LU-refine matrix upper permutation rhs initial-solution)))))

(save-test lu
 (test-lu-solve-dim *hilb2*)
 (test-lu-solve-dim *hilb3*)
 (test-lu-solve-dim *hilb4*)
 (test-lu-solve-dim *hilb12*)
 (test-lu-solve-dim *vander2*)
 (test-lu-solve-dim *vander3*)
 (test-lu-solve-dim *vander4*)
 (test-lu-solve-dim *vander12*)
 (test-lu-solve-dim (create-complex-matrix 7)))
