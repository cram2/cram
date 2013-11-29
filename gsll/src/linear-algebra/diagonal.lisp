;; Tridiagonal and Bidiagonal matrices
;; Liam Healy, Thu May  4 2006 - 15:43
;; Time-stamp: <2010-06-30 19:57:28EDT diagonal.lisp>
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

;;;;****************************************************************************
;;;; Tridiagonal Decomposition of Real Symmetric Matrices
;;;;****************************************************************************

;;; FDL
;;; A symmetric matrix A can be factorized by similarity
;;; transformations into the form 
;;; A = Q T Q^T
;;; where Q is an orthogonal matrix and T is a symmetric
;;; tridiagonal matrix.
;;; A hermitian matrix A can be factorized by similarity
;;; transformations into the form
;;; A = U T U^T where U is a unitary
;;; matrix and T is a real symmetric tridiagonal matrix.

(defmfun tridiagonal-decomposition ((A matrix) tau)
  (double-float "gsl_linalg_symmtd_decomp"
   complex-double-float "gsl_linalg_hermtd_decomp")
  (((mpointer A) :pointer) ((mpointer tau) :pointer))
  :definition :generic
  :element-types :doubles
  :inputs (A)
  :outputs (A tau)
  :documentation			; FDL
  "Factorizes the symmetric square matrix or hermitian matrix A into the
   symmetric tridiagonal decomposition Q T Q^T.  On output the
   diagonal and subdiagonal part of the input matrix A contain the
   tridiagonal matrix T.  The remaining lower triangular part of the
   input matrix contains the Householder vectors which, together with the
   Householder coefficients tau, encode the orthogonal matrix
   Q.  This storage scheme is the same as used by lapack.  The
   upper triangular part of A is not referenced.")

(defmfun tridiagonal-unpack ((A matrix) tau Q diag subdiag)
  (double-float "gsl_linalg_symmtd_unpack"
   complex-double-float "gsl_linalg_hermtd_unpack")
  (((mpointer A) :pointer) ((mpointer tau) :pointer)
   ((mpointer Q) :pointer) ((mpointer diag) :pointer)
   ((mpointer subdiag) :pointer))
  :definition :generic
  :element-types :doubles
  :inputs (A tau)
  :outputs (Q diag subdiag)
  :documentation			; FDL
  "Unpacks the encoded symmetric tridiagonal decomposition
  (A, tau) obtained from #'tridiagonal-decomposition into the
  orthogonal or unitary matrix Q, the vector of diagonal elements diag
  and the real vector of subdiagonal elements subdiag.")

(defmfun tridiagonal-unpack-T ((A matrix) diag subdiag)
  (double-float "gsl_linalg_symmtd_unpack_T"
   complex-double-float "gsl_linalg_hermtd_unpack_T")
  (((mpointer A) :pointer) ((mpointer diag) :pointer)
   ((mpointer subdiag) :pointer))
  :definition :generic
  :element-types :doubles
  :inputs (A)
  :outputs (diag subdiag)
  :documentation			; FDL
  "Unpack the diagonal and subdiagonal of the encoded symmetric
   tridiagonal decomposition (A, tau) obtained from
   #'tridiagonal-decomposition into the real vectors diag and
   subdiag.")

;;;;****************************************************************************
;;;; Bidiagonal
;;;;****************************************************************************

;;; FDL
;;; A general matrix A can be factorized by similarity
;;; transformations into the form A = U B V^T
;;; where U and V are orthogonal matrices and B is a
;;; N-by-N bidiagonal matrix with non-zero entries only on the
;;; diagonal and superdiagonal.  The size of U is M-by-N
;;; and the size of V is N-by-N.

(defmfun bidiagonal-decomposition (A tau-U tau-V)
  "gsl_linalg_bidiag_decomp"
  (((mpointer A) :pointer) ((mpointer tau-U) :pointer)
   ((mpointer tau-V) :pointer))
  :inputs (A)
  :outputs (A tau-U tau-V)
  :documentation			; FDL
  "Factorize the M-by-N matrix A into
   bidiagonal form U B V^T.  The diagonal and superdiagonal of the
   matrix B are stored in the diagonal and superdiagonal of A,
   The orthogonal matrices U and V are stored as compressed
   Householder vectors in the remaining elements of A.  The
   Householder coefficients are stored in the vectors tau-U and
   tau-V.  The length of tau-U must equal the number of
   elements in the diagonal of A and the length of tau-V should
   be one element shorter.")

(defmfun bidiagonal-unpack (A tau-U U tau-V V diag superdiag)
  "gsl_linalg_bidiag_unpack"
  (((mpointer A) :pointer)
   ((mpointer tau-U) :pointer) ((mpointer U) :pointer)
   ((mpointer tau-V) :pointer) ((mpointer V) :pointer)
   ((mpointer diag) :pointer) ((mpointer superdiag) :pointer))
  :inputs (A tau-U tau-V)
  :outputs (U V diag superdiag)
  :documentation			; FDL
  "Unpack the bidiagonal decomposition of A given by
   #'bidiagonal-decomposition (A, tau-U, tau-V)
   into the separate orthogonal matrices U, V, and the diagonal
   vector diag and superdiagonal superdiag.  Note that U
   is stored as a compact M-by-N orthogonal matrix satisfying
   U^T U = I for efficiency.")

;;; There is no 'diag and 'superdiag given for arguments to this function
(defmfun bidiagonal-unpack2 (A tau-U tau-V V)
  "gsl_linalg_bidiag_unpack2"
  (((mpointer A) :pointer) ((mpointer tau-U) :pointer)
   ((mpointer tau-V) :pointer) ((mpointer V) :pointer))
  :inputs (A tau-U tau-V)
  :outputs (A V)
  :documentation			; FDL
  "Unpack the bidiagonal decomposition of A given by
   #'bidiagonal-decomposition (A, tau-U, tau-V)
   into the separate orthogonal matrices U, V and the diagonal
   vector diag and superdiagonal superdiag.  The matrix U
   is stored in-place in A.")

(defmfun bidiagonal-unpack-diagonal-superdiagonal (A diag superdiag)
  "gsl_linalg_bidiag_unpack_B"
  (((mpointer A) :pointer) ((mpointer diag) :pointer)
   ((mpointer superdiag) :pointer))
  :inputs (A)
  :outputs (diag superdiag)
  :documentation			; FDL
  "Unpack the diagonal and superdiagonal of the bidiagonal
  decomposition of A given by #'bidiagonal-decomposition, into the
  diagonal vector diag and superdiagonal vector superdiag.")

;;;;****************************************************************************
;;;; Tridiagonal Systems
;;;;****************************************************************************

(defmfun solve-tridiagonal (diag superdiag subdiag b x)
  "gsl_linalg_solve_tridiag"
  (((mpointer diag) :pointer) ((mpointer superdiag) :pointer)
   ((mpointer subdiag) :pointer) ((mpointer b) :pointer)
   ((mpointer x) :pointer))
  :inputs (diag superdiag subdiag b)
  :outputs (x)
  :documentation			; FDL
  "Solve the general N-by-N system A x = b where A is tridiagonal
   (N >= 2). The super-diagonal and
   sub-diagonal vectors must be one element shorter
   than the diagonal vector diag.  The form of A for the 4-by-4
   case is
   A = ( d_0 e_0  0   0  )
       ( f_0 d_1 e_1  0  )
       (  0  f_1 d_2 e_2 )
       (  0   0  f_2 d_3 ).")

(defmfun solve-symmetric-tridiagonal (diag off-diagonal b x)
  "gsl_linalg_solve_symm_tridiag"
  (((mpointer diag) :pointer) ((mpointer off-diagonal) :pointer)
   ((mpointer b) :pointer) ((mpointer x) :pointer))
  :inputs (diag off-diagonal b)
  :outputs (x)
  :documentation			; FDL
  "Solve the general N-by-N system A x = b where A is
   symmetric tridiagonal (N >= 2).  The off-diagonal vector
   must be one element shorter than the diagonal vector diag.
   The form of A for the 4-by-4 case is
    A = ( d_0 e_0  0   0  )
        ( e_0 d_1 e_1  0  )
        (  0  e_1 d_2 e_2 )
        (  0   0  e_2 d_3 ).")

(defmfun solve-cyclic-tridiagonal (diag super-diag sub-diag b x)
  "gsl_linalg_solve_cyc_tridiag"
  (((mpointer diag) :pointer) ((mpointer super-diag) :pointer)
   ((mpointer sub-diag) :pointer)
   ((mpointer b) :pointer) ((mpointer x) :pointer))
  :inputs (diag super-diag sub-diag)
  :outputs (x)
  :documentation			; FDL
  "Solve the general N-by-N system A x = b where A is cyclic
   tridiagonal (N >= 3).  The cyclic super-diagonal and
   sub-diagonal vectors must have the same number of
   elements as the diagonal vector diag.  The form of A for the
   4-by-4 case is
     A = ( d_0 e_0  0  f_3 )
         ( f_0 d_1 e_1  0  )
         (  0  f_1 d_2 e_2 )
         ( e_3  0  f_2 d_3 ).")

(defmfun solve-symmetric-cyclic-tridiagonal (diag off-diagonal b x)
  "gsl_linalg_solve_symm_cyc_tridiag"
  (((mpointer diag) :pointer) ((mpointer off-diagonal) :pointer)
   ((mpointer b) :pointer) ((mpointer x) :pointer))
  :inputs (diag off-diagonal b)
  :outputs (x)
  :documentation			; FDL
  "Solve the general N-by-N system A x = b where A is symmetric
   cyclic tridiagonal (N >= 3).  The cyclic
   off-diagonal vector must have the same number of elements as the
   diagonal vector diag.  The form of A for the 4-by-4 case is
   shown below,
   A = ( d_0 e_0  0  e_3 )
       ( e_0 d_1 e_1  0  )
       (  0  e_1 d_2 e_2 )
       ( e_3  0  e_2 d_3 )")

;;;;****************************************************************************
;;;; Example
;;;;****************************************************************************

(defun solve-tridiagonal-example (&optional (n 6))
  "Solution differential equation
 y=1 with boundary conditions y(0)=(n-1)^2,  y(n-1)=0.

 The solution is the sequence: (n-1)^2, (n-2)^2, ... 9, 4, 2, 1, 0

 Desicretization of y leads to a tridiagonal system of equations
 (y_(i-1)-2_i+y_(i+1))/2 = 1 for 1 < i < (n - 2)

 The boundary conditions are implemented as 
 y(0)=(n-1)^2
 y(n-1)=0"
  (labels ((mvec (dimension &key initial-element)
	     "Shorthand for initializing gsll vectors"
	     (if initial-element
		 (grid:make-foreign-array 'double-float :dimensions dimension :initial-element initial-element)
		 (grid:make-foreign-array 'double-float :dimensions dimension))))
    (let ((x (mvec n))
	  (b (mvec n :initial-element 1d0))
	  (diag (mvec n :initial-element -1d0))
	  (superdiag (mvec (1- n) :initial-element 0.5d0))
	  (subdiag (mvec (1- n) :initial-element 0.5d0)))
      (setf
       ;; i=0 matrix elements
       (grid:gref diag 0) 1d0
       (grid:gref superdiag 0) 0d0
       (grid:gref b 0) (coerce (expt (1- n) 2) 'double-float)
       ;; i=n-1 matrix elements
       (grid:gref diag (1- n)) 1d0
       (grid:gref subdiag (- n 2)) 0d0
       (grid:gref b (1- n)) 0d0)
      (solve-tridiagonal diag superdiag subdiag b x))))
