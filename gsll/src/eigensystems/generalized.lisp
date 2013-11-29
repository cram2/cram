;; Real generalized eigensystems
;; Liam Healy 2009-02-16 12:55:04EST real-generalized.lisp
;; Time-stamp: <2010-06-29 22:15:24EDT generalized.lisp>
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

(in-package :gsl)

(defmobject eigen-gensymm
    "gsl_eigen_gensymm" ((n sizet))
    "symmetric generalized eigenvalue workspace"
    :gsl-version (1 10)
    :documentation			; FDL
    "Make a workspace for computing eigenvalues of n-by-n real
     generalized symmetric-definite eigensystems. The size of the workspace
     is O(2n).")

(defmobject eigen-gensymmv
    "gsl_eigen_gensymmv" ((n sizet))
    "symmetric generalized eigensystem workspace"
    :gsl-version (1 10)
    :documentation			; FDL
    "Make a workspace for computing eigenvalues and eigenvectors of
    n-by-n real generalized symmetric-definite eigensystems. The size
    of the workspace is O(4n).")

(defmobject eigen-genherm
    "gsl_eigen_genherm" ((n sizet))
    "hermitian generalized eigenvalue workspace"
    :gsl-version (1 10)
    :documentation			; FDL
    "Make a workspace for computing eigenvalues of n-by-n complex
    generalized hermitian-definite eigensystems. The size of the
    workspace is O(3n).")

(defmobject eigen-genhermv
    "gsl_eigen_genhermv" ((n sizet))
    "hermitian generalized eigensystem workspace"
    :gsl-version (1 10)
    :documentation			; FDL
    "Make a workspace for computing eigenvalues and eigenvectors of
    n-by-n complex generalized hermitian-definite eigensystems. The
    size of the workspace is O(5n).")

(defmfun eigenvalues-gensymm
    ((A matrix) (B matrix)
     &optional
     (eigenvalues (grid:make-foreign-array element-type :dimensions (dim0 A)))
     (ws (eltcase complex (make-eigen-genherm (dim0 A))
		  t (make-eigen-gensymm (dim0 A)))))
  (double-float "gsl_eigen_gensymm"
		complex-double-float "gsl_eigen_genherm")
  (((mpointer A) :pointer) ((mpointer B) :pointer)
   ((mpointer eigenvalues) :pointer) ((mpointer ws) :pointer))
  :gsl-version (1 10)
  :definition :generic
  :element-types :doubles
  :inputs (A B)
  :outputs (A B eigenvalues)
  :return (eigenvalues)
  :documentation			; FDL
  "Compute the eigenvalues of the real generalized symmetric-definite
  matrix pair (A, B), and stores them in eval, using the method
  outlined above. On output, B contains its Cholesky decomposition and
  A is destroyed.")

(defmfun eigenvalues-eigenvectors-gensymm
    ((A matrix) (B matrix)
     &optional
     (eigenvalues (grid:make-foreign-array element-type :dimensions (dim0 A)))
     (eigenvectors (grid:make-foreign-array element-type :dimensions (dimensions A)))
     (ws (eltcase complex (make-eigen-hermv (dim0 A))
		  t (make-eigen-symmv (dim0 A)))))
  (double-float "gsl_eigen_gensymmv"
		complex-double-float "gsl_eigen_genhermv")  
  (((mpointer A) :pointer) ((mpointer B) :pointer)
   ((mpointer eigenvalues) :pointer)
   ((mpointer eigenvectors) :pointer) ((mpointer ws) :pointer))
  :definition :generic
  :element-types :doubles
  :inputs (A B)
  :outputs (A B eigenvalues eigenvectors)
  :return (eigenvalues eigenvectors)
  :documentation			; FDL
  "Computes the eigenvalues and eigenvectors of the real generalized
  symmetric-definite matrix pair (A, B), and stores them in eval and
  evec respectively. The computed eigenvectors are normalized to have
  unit magnitude. On output, B contains its Cholesky decomposition and
  A is destroyed.")
