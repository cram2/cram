;; Eigenvectors and eigenvalues
;; Liam Healy, Sun May 21 2006 - 19:52
;; Time-stamp: <2012-01-13 12:01:32EST symmetric-hermitian.lisp>
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

;;; /usr/include/gsl/gsl_eigen.h

;;; Should symmetric matrices form a subclass of matrices, so that
;;; both eigenvalues and eigenvalues-nonsymm could be methods of the
;;; same function?

;;;;****************************************************************************
;;;; Workspace 
;;;;****************************************************************************

(defmobject eigen-symm
    "gsl_eigen_symm" ((n :sizet))
    "symmetric eigenvalue workspace"
    :documentation			; FDL
    "Make a workspace for computing eigenvalues of
  n-by-n real symmetric matrices.  The size of the workspace
  is O(2n).")

(defmobject eigen-symmv
    "gsl_eigen_symmv" ((n :sizet))
    "symmetric eigensystem workspace"
    :documentation			; FDL
    "Make a workspace for computing eigenvalues and
  eigenvectors of n-by-n real symmetric matrices.  The size of
  the workspace is O(4n).")

(defmobject eigen-herm
    "gsl_eigen_herm" ((n :sizet))
    "Hermitian eigenvalue workspace"	; FDL
    :documentation			; FDL
    "Make a workspace for computing eigenvalues of
  n-by-n complex Hermitian matrices.  The size of the workspace
  is O(3n).")

(defmobject eigen-hermv
    "gsl_eigen_hermv" ((n :sizet))
    "Hermitian eigensystem workspace"
    :documentation			; FDL
    "Make a workspace for computing eigenvalues and
  eigenvectors of n-by-n complex hermitian matrices.  The size of
  the workspace is O(5n).")

;;;;****************************************************************************
;;;; Eigenvalues and eigenvectors
;;;;****************************************************************************

(defmfun eigenvalues ((A grid:matrix)
     &optional
     (eigenvalues (grid:make-foreign-array element-type :dimensions (dim0 A)))
     (ws (eltcase complex (make-eigen-herm (dim0 A))
		  t (make-eigen-symm (dim0 A)))))
  (double-float "gsl_eigen_symm"
		complex-double-float "gsl_eigen_herm")
  (((mpointer A) :pointer)
   ((mpointer eigenvalues) :pointer) ((mpointer ws) :pointer))
  :definition :generic
  :element-types :doubles
  :inputs (A)
  :outputs (A eigenvalues)
  :return (eigenvalues)
  :documentation			; FDL
  "Eigenvalues of the real symmetric or complex hermitian matrix A.
  Additional workspace of the appropriate size and type must be
  provided in w.  The diagonal and lower triangular part of A are
  destroyed during the computation, but the strict upper triangular
  part is not referenced.  For the complex hermitian case, The
  imaginary parts of the diagonal are assumed to be zero and are not
  referenced.  The eigenvalues are stored in the vector eigenvalues and
  are unordered.")

(defmfun eigenvalues-eigenvectors ((A grid:matrix)
     &optional
     (eigenvalues (grid:make-foreign-array element-type :dimensions (dim0 A)))
     (eigenvectors (grid:make-foreign-array element-type :dimensions (grid:dimensions A)))
     (ws (eltcase complex (make-eigen-hermv (dim0 A))
		  t (make-eigen-symmv (dim0 A)))))
  (double-float "gsl_eigen_symmv"
		complex-double-float "gsl_eigen_hermv")  
  (((mpointer A) :pointer) ((mpointer eigenvalues) :pointer)
   ((mpointer eigenvectors) :pointer) ((mpointer ws) :pointer))
  :definition :generic
  :element-types :doubles
  :inputs (A)
  :outputs (A eigenvalues eigenvectors)
  :return (eigenvalues eigenvectors)
  :documentation			; FDL
  "The eigenvalues and eigenvectors of the real symmetric or complex
  hermitian matrix A.  Additional workspace of the appropriate size
  must be provided in w.  The diagonal and lower triangular part of A
  are destroyed during the computation, but the strict upper
  triangular part is not referenced.  For complex hermitian matrices,
  the imaginary parts of the diagonal are assumed to be zero and are
  not referenced.  The eigenvalues are stored in the vector
  eigenvalues and are unordered.  The corresponding eigenvectors are
  stored in the columns of the matrix eigenvectors.  For example, the
  eigenvector in the first column corresponds to the first eigenvalue.
  The eigenvectors are guaranteed to be mutually orthogonal and
  normalised to unit magnitude.")

;;;;****************************************************************************
;;;; Sorting Eigenvalues and Eigenvectors
;;;;****************************************************************************

(defmfun sort-eigenvalues-eigenvectors
    ((eigenvalues vector) (eigenvectors grid:matrix) sort-type)
  (double-float "gsl_eigen_symmv_sort"
   complex-double-float "gsl_eigen_hermv_sort")
  (((mpointer eigenvalues) :pointer) ((mpointer eigenvectors) :pointer)
   (sort-type eigen-sort-type))
  :definition :generic
  :element-types :doubles
  :inputs (eigenvalues eigenvectors)
  :outputs (eigenvalues eigenvectors)
  :documentation			; FDL
   "Simultaneously sort the eigenvalues stored in the vector
  eigenvalues and the corresponding real eigenvectors stored in the columns
  of the matrix eigenvectors into ascending or descending order according to
  the value of the parameter sort-type: :value-ascending,
  :value-descending, :absolute-ascending, :absolute-descending.")

;;;;****************************************************************************
;;;; Example
;;;;****************************************************************************
 
#|
;; When GSL 1.12 is in place and elt+ is supported for complex, this
;; can be expanded to :double-types, if we can create a hermitian
;; matrix.

(generate-all-array-tests eigensystems5 (double-float)
 (let ((m1 (array-default '(5 5))))
   (multiple-value-bind (eval evec)
       (eigenvalues-eigenvectors (elt+ m1 (matrix-transpose-copy m1)))
     (list (grid:copy-to eval) (grid:copy-to evec)))))
|#

(defun eigenvalue-eigenvectors-example ()
  (let ((evecs (grid:make-foreign-array 'double-float :dimensions '(3 3)))
	(evals (grid:make-foreign-array 'double-float :dimensions 3))
	(mat
	 (grid:make-foreign-array
	  'double-float
	  :initial-contents
	  '((20.0d0 -10.0d0 0.0d0)
	    (-10.0d0 30.0d0 0.0d0)
	    (0.0d0 0.0d0 40.0d0)))))
    (eigenvalues-eigenvectors mat evals evecs)
    (values (grid:copy-to evals) (grid:copy-to evecs))))

(save-test eigensystems
	   (eigenvalue-eigenvectors-example))

