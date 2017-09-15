;; Eigensystems for nonsymmetric real matrices
;; Liam Healy 2009-02-16 12:51:18EST nonsymmetric.lisp
;; Time-stamp: <2016-06-14 23:38:36EDT nonsymmetric.lisp>
;;
;; Copyright 2009, 2011, 2012, 2016 Liam M. Healy
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

(defmobject eigen-nonsymm
    "gsl_eigen_nonsymm" ((n :sizet))
    "non-symmetric eigenvalue workspace"
    :gsl-version (1 9)
    :documentation			; FDL
    "Make a workspace for computing eigenvalues of
  n-by-n real non-symmetric matrices.  The size of the workspace
  is O(2n).")

(defmobject eigen-nonsymmv
    "gsl_eigen_nonsymmv" ((n :sizet))
    "non-symmetric eigenvector and eigenvalue workspace"
    :gsl-version (1 9)
    :documentation			; FDL
    "Make a workspace for computing for computing eigenvalues and
    eigenvectors of n-by-n real nonsymmetric matrices. The size of the
    workspace is O(5n).")

(defmfun set-parameters-nonsymmetric
    (ws &optional compute-shur-form balance)
  "gsl_eigen_nonsymm_params"
  (((if compute-shur-form 1 0) :int)
   ((if balance 1 0) :int) ((mpointer ws) :pointer))
  :gsl-version (1 9)
  :c-return :void
  :export nil
  :index eigenvalues-nonsymm)

(defmfun eigenvalues-nonsymm
    (A
     &optional
     (eigenvalues
      (grid:make-foreign-array '(complex double-float) :dimensions (dim0 A)))
     (ws (make-eigen-nonsymm (dim0 A)))
     compute-shur-form balance shur-vectors
     &aux
     (sv
      (if (eql shur-vectors t)
	  (grid:make-foreign-array 'double-float :dimensions (grid:dimensions A))
	  shur-vectors)))
  ("gsl_eigen_nonsymm" "gsl_eigen_nonsymm_Z")
  ((((mpointer A) :pointer)
    ((mpointer eigenvalues) :pointer) ((mpointer ws) :pointer))
   (((mpointer A) :pointer)
    ((mpointer eigenvalues) :pointer) ((mpointer sv) :pointer)
    ((mpointer ws) :pointer)))
  :before
  ((set-parameters-nonsymmetric ws compute-shur-form balance))
  :gsl-version (1 9)
  :switch (shur-vectors)
  :inputs (A)
  :outputs (A eigenvalues)
  :return
  (eigenvalues
   (cffi:foreign-slot-value (mpointer ws) '(:struct gsl-nonsymm-ws) 'n-evals))
  :documentation			; FDL
  "Compute the eigenvalues of the real nonsymmetric matrix A and
  stores them in the vector 'eigenvalues. If T is desired, it is
  stored in the upper portion of A on output. Otherwise, on output,
  the diagonal of A will contain the 1-by-1 real eigenvalues and
  2-by-2 complex conjugate eigenvalue systems, and the rest of A is
  destroyed. In rare cases, this function may fail to find all
  eigenvalues. If this happens, a warning is signalled and the number
  of converged eigenvalues is returned as a second value. The
  converged eigenvalues are stored in the beginning of eval.

  If compute-shur-form is true, the full Schur form T will be computed.
  If it is set to nil, T will not be computed (this is
  the default setting). Computing the full Schur form requires
  approximately 1.5-2 times the number of flops.

  If balance is true, a balancing transformation is applied to the
  matrix prior to computing eigenvalues. This transformation is
  designed to make the rows and columns of the matrix have comparable
  norms, and can result in more accurate eigenvalues for matrices
  whose entries vary widely in magnitude. See Balancing for more
  information. Note that the balancing transformation does not
  preserve the orthogonality of the Schur vectors, so if you wish to
  compute the Schur vectors with you will obtain the Schur vectors of
  the balanced matrix instead of the original matrix. The relationship
  will be

          T = Q^t D^(-1) A D Q

  where Q is the matrix of Schur vectors for the balanced matrix, and D
  is the balancing transformation. Then this function will compute
  a matrix Z which satisfies

          T = Z^(-1) A Z

  with Z = D Q. Note that Z will not be orthogonal. For this reason,
  balancing is not performed by default.")

(defmfun eigenvalues-eigenvectors-nonsymm
    (A
     &optional
     (eigenvalues
      (grid:make-foreign-array '(complex double-float) :dimensions (dim0 A)))
     (eigenvectors
      (grid:make-foreign-array  '(complex double-float) :dimensions (grid:dimensions A)))
     (ws (make-eigen-nonsymmv (dim0 A)))
     shur-vectors
     &aux
     (sv
      (if (eql shur-vectors t)
	  (grid:make-foreign-array 'double-float :dimensions (grid:dimensions A))
	  shur-vectors)))
  ("gsl_eigen_nonsymmv" "gsl_eigen_nonsymmv_Z")
  ((((mpointer A) :pointer)
    ((mpointer eigenvalues) :pointer) ((mpointer eigenvectors) :pointer)
    ((mpointer ws) :pointer))
   (((mpointer A) :pointer)
    ((mpointer eigenvalues) :pointer) ((mpointer eigenvectors) :pointer)
    ((mpointer ws) :pointer) ((mpointer sv) :pointer)))
  :gsl-version (1 9)
  :switch (shur-vectors)
  :inputs (A)
  :outputs (A eigenvalues eigenvectors)
  :return (eigenvalues eigenvectors)
  :documentation			; FDL
  "Compute eigenvalues and right eigenvectors of the n-by-n real
  nonsymmetric matrix A. It first calls #'eigenvalues-nonsymm to
  compute the eigenvalues, Schur form T, and Schur vectors. Then it
  finds eigenvectors of T and backtransforms them using the Schur
  vectors. The Schur vectors are destroyed in the process, but can be
  saved by specifying binding shur-vectors to a vector of length n, or
  t to have it automatically made.  The computed eigenvectors are
  normalized to have unit magnitude. On output, the upper portion of A
  contains the Schur form T.  If #'eigenvalues-nonsymm fails, no
  eigenvectors are computed, and an error code is returned.")

