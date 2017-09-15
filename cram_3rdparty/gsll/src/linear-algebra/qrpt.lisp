;; QR with column pivoting
;; Liam Healy, Fri Apr 28 2006 - 16:53
;; Time-stamp: <2011-04-23 17:15:46EDT qrpt.lisp>
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

;;; /usr/include/gsl/gsl_linalg.h

(defmfun QRPT-decomposition
    (A
     &optional
     (tau (grid:make-foreign-array 'double-float :dimensions (min (dim0 A) (dim1 A))))
     (permutation (make-permutation (dim1 A)))
     (norm (grid:make-foreign-array 'double-float :dimensions (dim1 A))))
  "gsl_linalg_QRPT_decomp"
  (((mpointer A) :pointer) ((mpointer tau) :pointer)
   ((mpointer permutation) :pointer)
   (signum (:pointer :int)) ((mpointer norm) :pointer))
  :inputs (A)
  :outputs (A tau permutation norm)
  :return (A tau permutation signum)
  :documentation			; FDL
  "Factorizes the M-by-N matrix A into
   the QRP^T decomposition A = Q R P^T.  On output the
   diagonal and upper triangular part of the input matrix contain the
   matrix R.  The permutation matrix P is stored in the
   permutation.  The sign of the permutation is given by
   signum. It has the value (-1)^n, where n is the
   number of interchanges in the permutation. The vector tau and the
   columns of the lower triangular part of the matrix A contain the
   Householder coefficients and vectors which encode the orthogonal matrix
   Q.  The vector tau must be of length k=min(M,N). The
   matrix Q is related to these components by, Q = Q_k ... Q_2 Q_1
   where Q_i = I - tau_i v_i v_i^T and v_i is the Householder vector
   v_i = (0,...,1,A(i+1,i),A(i+2,i),...,A(m,i)). This is the same storage scheme
   as used by lapack.  The vector norm is a workspace of length
   N used for column pivoting.

   The algorithm used to perform the decomposition is Householder QR with
   column pivoting (Golub & Van Loan, Matrix Computations, Algorithm
   5.4.1).")

(defmfun QRPT-decomposition*
    (A
     &optional
     (q (grid:make-foreign-array 'double-float :dimensions (list (dim0 A) (dim0 A))))
     (r (grid:make-foreign-array 'double-float :dimensions (grid:dimensions A)))
     (tau (grid:make-foreign-array 'double-float :dimensions (min (dim0 A) (dim1 A))))
     (permutation (make-permutation (dim1 A)))
     (norm (grid:make-foreign-array 'double-float :dimensions (dim1 A))))
  "gsl_linalg_QRPT_decomp2"
  (((mpointer A) :pointer) ((mpointer q) :pointer)
   ((mpointer r) :pointer) ((mpointer tau) :pointer)
   ((mpointer permutation) :pointer) (signum (:pointer :int))
   ((mpointer norm) :pointer))
  :inputs (A)
  :outputs (q r norm)
  :return (q r permutation signum)
  :documentation			; FDL
  "Factorize the matrix A into the decomposition
  A = Q R P^T without modifying A itself and storing the
  output in the separate matrices q and r.")

(defmfun QRPT-solve
    (QR tau permutation b &optional x-spec
       &aux
       (x (grid:ensure-foreign-array x-spec (grid:dimensions b))))
  ("gsl_linalg_QRPT_svx" "gsl_linalg_QRPT_solve")
  ((((mpointer QR) :pointer) ((mpointer tau) :pointer)
    ((mpointer permutation) :pointer) ((mpointer b) :pointer))
   (((mpointer QR) :pointer) ((mpointer tau) :pointer)
    ((mpointer permutation) :pointer)
    ((mpointer b) :pointer) ((mpointer x) :pointer)))
  :inputs (QR tau permutation b)
  :outputs (x)
  :return ((or x b))
  :documentation			; FDL
  "Solve the square system A x = b using the QRP^T decomposition of A
   into (QR, tau, permutation) given by #'QRPT-decomposition.  If x-spec is
   NIL (default), the solution will replace b.  If x-spec is T, then
   an array will be created and the solution returned in it.  If
   x-spec is a grid:foreign-array, the solution will be returned in it.  If x-spec
   is non-NIL, on output the solution is stored in x and b is not
   modified.  The solution is returned from the function call.")

(defmfun QRPT-QRsolve
    (Q R permutation b
       &optional (x (grid:make-foreign-array 'double-float :dimensions (dim0 b))))
  "gsl_linalg_QRPT_QRsolve"
  (((mpointer Q) :pointer) ((mpointer R) :pointer)
   ((mpointer permutation) :pointer)
   ((mpointer b) :pointer) ((mpointer x) :pointer))
  :inputs (Q R permutation b)
  :outputs (x)
  :documentation			; FDL
  "Solve the square system R P^T x = Q^T b for
   x. It can be used when the QR decomposition of a matrix is
   available in unpacked form as (Q, R).")

(defmfun QRPT-update (Q R permutation w v)
  "gsl_linalg_QRPT_update"
  (((mpointer Q) :pointer) ((mpointer R) :pointer)
   ((mpointer permutation) :pointer)
   ((mpointer w) :pointer) ((mpointer v) :pointer))
  :inputs (Q R permutation w v)
  :outputs (w Q R)
  :return (Q R)
  :documentation			; FDL
  "Perform a rank-1 update w v^T of the QRP^T
   decomposition (Q, R, p). The update is given by
   Q'R' = Q R + w v^T where the output matrices Q' and
   R' are also orthogonal and right triangular. Note that w is
   destroyed by the update. The permutation is not changed.")

(defmfun QRPT-Rsolve
    (QR permutation b &optional x-spec
       &aux
       (x (grid:ensure-foreign-array x-spec (grid:dimensions b))))
  ("gsl_linalg_QRPT_Rsvx" "gsl_linalg_QRPT_Rsolve")
  ((((mpointer QR) :pointer) ((mpointer permutation) :pointer)
    ((mpointer b) :pointer))
   (((mpointer QR) :pointer) ((mpointer permutation) :pointer)
    ((mpointer b) :pointer) ((mpointer x) :pointer)))
  :inputs (QR permutation b)
  :outputs (x b)
  :return ((or x b))
  :documentation			; FDL
  "Solve the triangular system R P^T x = b in-place for the N-by-N
  matrix R contained in QR. On input x should contain the right-hand
  side b, which is replaced by the solution on output.  If x-spec is
  NIL (default), the solution will replace b.  If x-spec is T, then an
  array will be created and the solution returned in it.  If x-spec is
  a grid:foreign-array, the solution will be returned in it.  If x-spec is
  non-NIL, on output the solution is stored in x and b is not
  modified.  The solution is returned from the function call.")

;;; Examples and unit test, from linalg/test.c

(defun test-qrpt-solve-dim (matrix)
  "Solve the linear equation using QRPT with the supplied matrix and
   a right-hand side vector which is the reciprocal of one more than
   the index."
  (let ((dim (dim0 matrix)))
    (multiple-value-bind (QRPT tau permutation)
	(QRPT-decomposition (copy matrix))
      (QRPT-solve QRPT tau permutation (create-rhs-vector dim) T))))

(defun test-qrpt-qrsolve-dim (matrix)
  "Solve the linear equation using QRPT with the supplied matrix and
   a right-hand side vector which is the reciprocal of one more than
   the index."
  (let ((dim (dim0 matrix)))
    (multiple-value-bind (Q R permutation)
	(QRPT-decomposition* (copy matrix))
      (QRPT-QRsolve Q R permutation (create-rhs-vector dim)))))

(defun test-qrpt-decomp-dim (matrix)
  "Solve the QRPT decomposition with the supplied
   matrix and a right-hand side vector which is the reciprocal of one
   more than the index."
  (multiple-value-bind (QRPT tau permutation)
      (QRPT-decomposition (copy matrix))
    (multiple-value-bind (Q R)
	(QR-unpack QRPT tau)
      (let* ((qr (matrix-product Q R))
	     (ans (grid:make-foreign-array
		   'double-float :dimensions (grid:dimensions qr)
		   ;; It shouldn't be necessary to initialize values
		   ;; because we are going to setf every row,
		   ;; Sat Dec 26 2009.
		   :initial-element 0)))
	(dotimes (i (dim0 qr) ans)
	  (setf (row ans i) (permute-inverse permutation (row qr i))))))))

(save-test qrpt
 ;; test_QRPT_solve
 (test-qrpt-solve-dim *hilb2*)
 (test-qrpt-solve-dim *hilb3*)
 (test-qrpt-solve-dim *hilb4*)
 (test-qrpt-solve-dim *hilb12*)
 (test-qrpt-solve-dim *vander2*)
 (test-qrpt-solve-dim *vander3*)
 (test-qrpt-solve-dim *vander4*)
 (test-qrpt-solve-dim *vander12*)
 ;; test_QRPT_QRsolve
 (test-qrpt-qrsolve-dim *hilb2*)
 (test-qrpt-qrsolve-dim *hilb3*)
 (test-qrpt-qrsolve-dim *hilb4*)
 (test-qrpt-qrsolve-dim *hilb12*)
 (test-qrpt-qrsolve-dim *vander2*)
 (test-qrpt-qrsolve-dim *vander3*)
 (test-qrpt-qrsolve-dim *vander4*)
 (test-qrpt-qrsolve-dim *vander12*)
 ;; test_QRPT_decomp
 (test-qrpt-decomp-dim *m35*)
 (test-qrpt-decomp-dim *m53*)
 (test-qrpt-decomp-dim *hilb2*)
 (test-qrpt-decomp-dim *hilb3*)
 (test-qrpt-decomp-dim *hilb4*)
 (test-qrpt-decomp-dim *hilb12*)
 (test-qrpt-decomp-dim *vander2*)
 (test-qrpt-decomp-dim *vander3*)
 (test-qrpt-decomp-dim *vander4*)
 (test-qrpt-decomp-dim *vander12*))
