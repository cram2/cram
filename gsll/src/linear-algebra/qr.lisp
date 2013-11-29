;; QR decomposition
;; Liam Healy 2008-02-17 11:05:20EST qr.lisp
;; Time-stamp: <2010-06-30 19:57:28EDT qr.lisp>
;;
;; Copyright 2008, 2009 Liam M. Healy
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

;;; FDL
;;; A general rectangular M-by-N matrix A has a
;;; QR decomposition into the product of an orthogonal
;;; M-by-M square matrix Q (where Q^T Q = I) and
;;; an M-by-N right-triangular matrix R, A = Q R.

;;; This decomposition can be used to convert the linear system A x = b
;;; into the triangular system R x = Q^T b, which can be solved by
;;; back-substitution. Another use of the QR decomposition is to
;;; compute an orthonormal basis for a set of vectors. The first N
;;; columns of Q form an orthonormal basis for the range of A,
;;; ran(A), when A has full column rank.

(defmfun QR-decomposition
    (A
     &optional
     (tau (grid:make-foreign-array 'double-float :dimensions (min (dim0 A) (dim1 A)))))
  "gsl_linalg_QR_decomp"
  (((mpointer A) :pointer) ((mpointer tau) :pointer))
  :inputs (A)
  :outputs (A tau)
  :documentation 			; FDL
  "Factorize the M-by-N matrix A into the QR decomposition A = Q R.
   On output the diagonal and
   upper triangular part of the input matrix contain the matrix
   R.  The vector tau and the columns of the lower triangular
   part of the matrix A contain the Householder coefficients and
   Householder vectors which encode the orthogonal matrix Q.  The
   vector tau must be of length k=min(M,N). The matrix
   Q is related to these components by, Q = Q_k ... Q_2 Q_1
   where Q_i = I - tau_i v_i v_i^T and v_i is the
   Householder vector v_i = (0,...,1,A(i+1,i),A(i+2,i),...,A(m,i)).
   This is the same storage scheme as used by lapack.

   The algorithm used to perform the decomposition is Householder QR (Golub
   & Van Loan, Matrix Computations, Algorithm 5.2.1).")

(defmfun QR-solve
    (QR tau b &optional x-spec
	&aux
	(x (grid:make-foreign-array-or-default x-spec (dimensions b) t)))
  ("gsl_linalg_QR_svx" "gsl_linalg_QR_solve")
  ((((mpointer QR) :pointer) ((mpointer tau) :pointer)
    ((mpointer b) :pointer))
   (((mpointer QR) :pointer) ((mpointer tau) :pointer)
    ((mpointer b) :pointer) ((mpointer x) :pointer)))
  :inputs (QR tau b)
  :outputs (x)
  :return ((or x b))
  :documentation			; FDL
  "Solve the square system A x = b using the QR decomposition of A
   into (QR, tau) given by QR-decomp. The least-squares solution for
   rectangular systems can be found using QR-lssolve.  If x-spec is
   NIL (default), the solution will replace b.  If x-spec is T, then
   an array will be created and the solution returned in it.  If
   x-spec is a grid:foreign-array, the solution will be returned in it.  If x-spec
   is non-NIL, on output the solution is stored in x and b is not
   modified.  The solution is returned from the function call.")

(defmfun QR-solve-least-squares
    (QR tau b
	&optional
	 (x (grid:make-foreign-array 'double-float :dimensions (dim1 QR)))
	 (residual (grid:make-foreign-array 'double-float :dimensions (dim0 QR))))
  "gsl_linalg_QR_lssolve"
  (((mpointer QR) :pointer) ((mpointer tau) :pointer)
   ((mpointer b) :pointer) ((mpointer x) :pointer)
   ((mpointer residual) :pointer))
  :inputs (QR tau b)
  :outputs (x residual)
  :documentation			; FDL
  "The least squares solution to the overdetermined system A x = b
   where the matrix A has more rows than columns.  The least squares
   solution minimizes the Euclidean norm of the residual, ||Ax -
   b||.The routine uses the QR decomposition of A into (QR, tau) given
   by #'QR-decomposition.  The solution is returned in x.  The
   residual is computed as a by-product and stored in residual.")

(defmfun QR-QTvector (QR tau v)
  "gsl_linalg_QR_QTvec"
  (((mpointer QR) :pointer) ((mpointer tau) :pointer)
   ((mpointer v) :pointer))
  :inputs (QR tau)
  :outputs (v)
  :documentation			; FDL
  "Apply the matrix Q^T encoded in the decomposition
   (QR, tau) to the vector v, storing the result Q^T v in v.
   The matrix multiplication is carried out directly using
   the encoding of the Householder vectors without needing to form the full
   matrix Q^T.")

(defmfun QR-Qvector (QR tau v)
  "gsl_linalg_QR_Qvec"
  (((mpointer QR) :pointer) ((mpointer tau) :pointer)
   ((mpointer v) :pointer))
  :inputs (QR tau)
  :outputs (v)
  :documentation			; FDL
  "Apply the matrix Q encoded in the decomposition
   (QR, tau) to the vector v, storing the result Q v in v.
   The matrix multiplication is carried out directly using
   the encoding of the Householder vectors without needing to form the full
   matrix Q.")

(defmfun QR-Rsolve
    (QR b &optional x-spec
       &aux
       (x (grid:make-foreign-array-or-default x-spec (dimensions b) t)))
  ("gsl_linalg_QR_Rsvx" "gsl_linalg_QR_Rsolve")
  ((((mpointer QR) :pointer) ((mpointer b) :pointer))
   (((mpointer QR) :pointer) ((mpointer b) :pointer) ((mpointer x) :pointer)))
  :inputs (QR b x)
  :outputs (b x)
  :return ((or x b))
  :documentation			; FDL
  "Solve the triangular system R x = b for x.  It may be useful if the
   product b' = Q^T b has already been computed using QR-QTvec.  If
   x-spec is NIL (default), the solution will replace b.  If x-spec is
   T, then an array will be created and the solution returned in it.
   If x-spec is a grid:foreign-array, the solution will be returned in it.  If
   x-spec is non-NIL, on output the solution is stored in x and b is
   not modified.  The solution is returned from the function call.")

(defmfun QR-unpack
    (QR tau
     &optional
     (Q (grid:make-foreign-array 'double-float :dimensions (list (dim0 QR) (dim0 QR))))
     (R (grid:make-foreign-array 'double-float :dimensions (dimensions QR))))
  "gsl_linalg_QR_unpack"
  (((mpointer QR) :pointer) ((mpointer tau) :pointer)
   ((mpointer Q) :pointer) ((mpointer R) :pointer))
  :inputs (QR tau)
  :outputs (Q R)
  :documentation			; FDL
  "Unpack the encoded QR decomposition
  (QR, tau) into the matrices Q and R where
  Q is M-by-M and R is M-by-N.")

(defmfun QR-QRsolve
    (Q R b &optional (x (grid:make-foreign-array 'double-float :dimensions (dim0 b))))
  "gsl_linalg_QR_QRsolve"
  (((mpointer Q) :pointer) ((mpointer R) :pointer)
   ((mpointer b) :pointer) ((mpointer x) :pointer))
  :inputs (Q R b)
  :outputs (x)
  :documentation			; FDL
  "Solves the system R x = Q^T b for x.  It can
  be used when the QR decomposition of a matrix is available in
  unpacked form as Q, R).")

(defmfun QR-update (Q R w v)
  "gsl_linalg_QR_update"
  (((mpointer Q) :pointer) ((mpointer R) :pointer)
   ((mpointer w) :pointer) ((mpointer v) :pointer))
  :inputs (Q R w v)
  :outputs (w Q R)
  :return (Q R)
  :documentation			; FDL
  "Perform a rank-1 update w v^T of the QR
  decomposition (Q, R). The update is given by Q'R' = Q R + w v^T
  where the output matrices Q' and R' are also
  orthogonal and right triangular. Note that w is destroyed by the
  update.")

(defmfun R-solve
    (R b &optional x-spec
       &aux
       (x (grid:make-foreign-array-or-default x-spec (dimensions b) t)))
  ("gsl_linalg_R_svx" "gsl_linalg_R_solve")
  ((((mpointer R) :pointer) ((mpointer b) :pointer))
   (((mpointer R) :pointer) ((mpointer b) :pointer) ((mpointer x) :pointer)))
  :inputs (R b x)
  :outputs (x)
  :return ((or x b))
  :documentation			; FDL
  "Solve the triangular system R x = b in-place. On input x should
  contain the right-hand side b, which is replaced by the solution on
  output.  If x-spec is NIL (default), the solution will replace b.
  If x-spec is T, then an array will be created and the solution
  returned in it.  If x-spec is a grid:foreign-array, the solution will be
  returned in it.  If x-spec is non-NIL, on output the solution is
  stored in x and b is not modified.  The solution is returned from
  the function call.")

;;; Examples and unit test, from linalg/test.c

(defun test-qr-solve-dim (matrix)
  "Solve the linear equation using QR with the supplied matrix and
   a right-hand side vector which is the reciprocal of one more than
   the index."
  (let ((dim (dim0 matrix)))
    (multiple-value-bind (QR tau)
	(QR-decomposition (copy matrix))
      (QR-solve QR tau (create-rhs-vector dim) T))))

(defun test-qr-qrsolve-dim (matrix)
  "Solve the linear equation using QR with the supplied matrix and
   a right-hand side vector which is the reciprocal of one more than
   the index."
  (let ((dim (dim0 matrix)))
    (multiple-value-bind (QR tau)
	(QR-decomposition (copy matrix))
      (multiple-value-bind (Q R)
	  (QR-unpack QR tau)
	(QR-QRsolve Q R (create-rhs-vector dim))))))

(defun test-qr-lssolve-dim (matrix)
  "Solve the linear equation using QR least squares with the supplied
   matrix and a right-hand side vector which is the reciprocal of one
   more than the index.  Returns the solution and the residual."
  (let ((dim (dim0 matrix)))
    (multiple-value-bind (QR tau)
	(QR-decomposition (copy matrix))
      ;; Residual not checked.
      (QR-solve-least-squares QR tau (create-rhs-vector dim)))))

(defun test-qr-decomp-dim (matrix)
  "Solve the QR decomposition with the supplied
   matrix and a right-hand side vector which is the reciprocal of one
   more than the index."
  (multiple-value-bind (QR tau)
      (QR-decomposition (copy matrix))
    (multiple-value-bind (Q R)
	(QR-unpack QR tau)
      (matrix-product Q R))))

(defun test-qr-update-dim (matrix)
  "Test QR rank-1 update; this should return a matrix with all
   elements near zero."
  (let* ((dim0 (dim0 matrix)) (dim1 (dim1 matrix))
	 (u (create-matrix (lambda (i) (sin (1+ i))) dim0 nil))
	 (v (create-matrix
	     (lambda (i) (+ (cos (+ 2 i)) (sin (+ 3 (expt i 2)))))
	     dim1 nil))
	 (qr1
	  (create-matrix
	   (lambda (i j) (+ (grid:gref matrix i j) (* (grid:gref u i) (grid:gref v j))))
	   dim0 dim1))
	 (qr2 (copy matrix))
	 (w (grid:make-foreign-array 'double-float :dimensions dim0 :initial-element 0)))
    (multiple-value-bind (QR2 tau)
	(QR-decomposition qr2)
      (multiple-value-bind (Q2 R2)
	  (QR-unpack QR2 tau)
	;; compute w = Q^T u
	(matrix-product Q2 u w 1.0d0 0.0d0 :trans)
	(QR-update Q2 R2 w v)
	(matrix-product Q2 R2 qr2 1.0d0 0.0d0)
	(elt- qr1 qr2)))))

(save-test qr
 ;; test_QR_solve
 (test-qr-solve-dim *hilb2*)
 (test-qr-solve-dim *hilb3*)
 (test-qr-solve-dim *hilb4*)
 (test-qr-solve-dim *hilb12*)
 (test-qr-solve-dim *vander2*)
 (test-qr-solve-dim *vander3*)
 (test-qr-solve-dim *vander4*)
 (test-qr-solve-dim *vander12*)
 ;; test_QR_QRsolve
 (test-qr-qrsolve-dim *hilb2*)
 (test-qr-qrsolve-dim *hilb3*)
 (test-qr-qrsolve-dim *hilb4*)
 (test-qr-qrsolve-dim *hilb12*)
 (test-qr-qrsolve-dim *vander2*)
 (test-qr-qrsolve-dim *vander3*)
 (test-qr-qrsolve-dim *vander4*)
 (test-qr-qrsolve-dim *vander12*)
 ;; test_QR_lssolve
 (test-qr-lssolve-dim *m53*)
 (test-qr-lssolve-dim *hilb2*)
 (test-qr-lssolve-dim *hilb3*)
 (test-qr-lssolve-dim *hilb4*)
 (test-qr-lssolve-dim *hilb12*)
 (test-qr-lssolve-dim *vander2*)
 (test-qr-lssolve-dim *vander3*)
 (test-qr-lssolve-dim *vander4*)
 (test-qr-lssolve-dim *vander12*)
 ;; test_QR_decomp
 (test-qr-decomp-dim *m35*)
 (test-qr-decomp-dim *m53*)
 (test-qr-decomp-dim *hilb2*)
 (test-qr-decomp-dim *hilb3*)
 (test-qr-decomp-dim *hilb4*)
 (test-qr-decomp-dim *hilb12*)
 (test-qr-decomp-dim *vander2*)
 (test-qr-decomp-dim *vander3*)
 (test-qr-decomp-dim *vander4*)
 (test-qr-decomp-dim *vander12*)
 ;; test_QR_update
 (test-qr-update-dim *m35*)
 (test-qr-update-dim *m53*)
 (test-qr-update-dim *hilb2*)
 (test-qr-update-dim *hilb3*)
 (test-qr-update-dim *hilb4*)
 (test-qr-update-dim *hilb12*)
 (test-qr-update-dim *vander2*)
 (test-qr-update-dim *vander3*)
 (test-qr-update-dim *vander4*)
 (test-qr-update-dim *vander12*))
