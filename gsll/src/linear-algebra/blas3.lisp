;; BLAS level 3, Matrix-matrix operations
;; Liam Healy, Wed Apr 26 2006 - 21:08
;; Time-stamp: <2010-07-07 14:25:00EDT blas3.lisp>
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

;;; /usr/include/gsl/gsl_blas.h

;;;;****************************************************************************
;;;; Options
;;;;****************************************************************************

#+fsbv
(fsbv:defcenum-aux cblas-side)

;;;;****************************************************************************
;;;; Functions
;;;;****************************************************************************

(defmfun matrix-product
    ((A matrix) (B matrix)
     &optional
     C
     (alpha 1) (beta 1) (TransA :notrans) (TransB :notrans)
     &aux
     (Carr
      (or C
	  (grid:make-foreign-array element-type :dimensions (matrix-product-dimensions A B)
		       :initial-element 0))))
  ("gsl_blas_" :type "gemm")
  ((TransA cblas-transpose) (TransB cblas-transpose)
   (alpha :element-c-type) ((mpointer A) :pointer)
   ((mpointer B) :pointer) (beta :element-c-type) ((mpointer Carr) :pointer))
  :definition :methods
  :element-types #+fsbv :float-complex #-fsbv :float
  :inputs (A B Carr)
  :outputs (Carr))

(defmfun matrix-product-symmetric
    ((A matrix) (B matrix)
     &optional C (alpha 1) (beta 1) (uplo :upper) (side :left)
     &aux
     (Carr
      (or C
	  (grid:make-foreign-array element-type :dimensions (matrix-product-dimensions A B)
		       :initial-element 0))))
  ("gsl_blas_" :type "symm")
  ((side cblas-side) (uplo cblas-uplo) (alpha :element-c-type)
   ((mpointer A) :pointer) ((mpointer B) :pointer)
   (beta :element-c-type) ((mpointer Carr) :pointer)) 
  :definition :methods
  :element-types #+fsbv :float-complex #-fsbv :float
  :inputs (A B Carr)
  :outputs (Carr))

#+fsbv
(defmfun matrix-product-hermitian
    ((A matrix) (B matrix)
     &optional
     (C (grid:make-foreign-array element-type :dimensions (matrix-product-dimensions A B)
		     :initial-element 0))
     (alpha 1) (beta 1) (uplo :upper) (side :left))
  ;; This always signals an error because you can't pass a
  ;; struct in CFFI yet.
  ("gsl_blas_" :type "hemm")
  ((side cblas-side) (uplo cblas-uplo) (alpha :element-c-type)
   ((mpointer A) :pointer) ((mpointer B) :pointer)
   (beta :element-c-type) ((mpointer C) :pointer))
  :definition :methods
  :element-types :complex
  :inputs (A B C)
  :outputs (C))

(defmfun matrix-product-triangular
    ((A matrix) (B matrix)
     &optional (alpha 1) (uplo :upper) (TransA :notrans)
     (diag :nonunit) (side :left))
  ("gsl_blas_" :type "trmm")
  ((side cblas-side) (uplo cblas-uplo) (TransA cblas-transpose)
   (diag cblas-diag)
   (alpha :element-c-type) ((mpointer A) :pointer) ((mpointer B) :pointer))
  :definition :methods
  :element-types #+fsbv :float-complex #-fsbv :float
  :inputs (A B)
  :outputs (B))

(defmfun inverse-matrix-product
    ((A matrix) (B matrix)
     &optional (alpha 1) (uplo :upper) (TransA :notrans)
     (diag :nonunit) (side :left))
  ;; This signals an error for complex arguments because you can't pass a
  ;; struct in CFFI yet.
  ("gsl_blas_" :type "trsm")
  ((side cblas-side) (uplo cblas-uplo)
   (TransA cblas-transpose) (diag cblas-diag)
   (alpha :element-c-type) ((mpointer A) :pointer) ((mpointer B) :pointer))
  :definition :methods
  :element-types #+fsbv :float-complex #-fsbv :float
  :inputs (A B)
  :outputs (B))

(defmfun symmetric-rank-1-update
    ((A matrix) (C matrix)
     &optional (alpha 1) (beta 1) (uplo :upper) (trans :notrans))
  ("gsl_blas_" :type "syrk")
  ((uplo cblas-uplo) (trans cblas-transpose)
   (alpha :element-c-type) ((mpointer A) :pointer)
   (beta :element-c-type) ((mpointer C) :pointer))
  :definition :methods
  :element-types #+fsbv :float-complex #-fsbv :float
  :inputs (A C)
  :outputs (C))

#+fsbv
(defmfun hermitian-rank-1-update
    ((A matrix) (C matrix)
     &optional (alpha 1) (beta 1) (uplo :upper) (trans :notrans))
  ("gsl_blas_" :type "herk")
  ((uplo cblas-uplo) (trans cblas-transpose)
   (alpha :element-c-type) ((mpointer A) :pointer)
   (beta :element-c-type) ((mpointer C) :pointer))
  :definition :methods
  :element-types :complex
  :inputs (A C)
  :outputs (C))

(defmfun symmetric-rank-2-update
    ((A matrix) (B matrix) (C matrix)
     &optional (alpha 1) (beta 1) (uplo :upper) (trans :notrans))
  ("gsl_blas_" :type "syr2k")
  ((uplo cblas-uplo) (trans cblas-transpose)
   (alpha :element-c-type) ((mpointer A) :pointer)
   ((mpointer B) :pointer) (beta :element-c-type)
   ((mpointer C) :pointer))
  :definition :methods 
  :element-types #+fsbv :float-complex #-fsbv :float
  :inputs (A B C)
  :outputs (C))

#+fsbv
(defmfun hermitian-rank-2-update
    ((A matrix) (B matrix) (C matrix)
     &optional (alpha 1) (beta 1) (uplo :upper) (trans :notrans))
  ("gsl_blas_" :type "her2k")
  ((uplo cblas-uplo) (trans cblas-transpose)
   (alpha :element-c-type) ((mpointer A) :pointer)
   ((mpointer B) :pointer) (beta :element-c-type)
   ((mpointer C) :pointer))
  :definition :methods
  :element-types :complex
  :inputs (A B C)
  :outputs (C))

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(generate-all-array-tests matrix-product #+fsbv :float-complex #-fsbv :float
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3)))
       (m3 (array-default '(3 3)))
       (s1 (scalar-default))
       (s2 (scalar-default)))
   (grid:copy-to (matrix-product m1 m2 m3 s1 s2))))

(generate-all-array-tests matrix-product-nonsquare :float-complex
 (let ((m1 (array-default '(2 3)))
       (m2 (array-default '(3 2))))
   (grid:copy-to (matrix-product m1 m2))))

(generate-all-array-tests matrix-product-triangular
			  #+fsbv :float-complex #-fsbv :float
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3)))
       (s1 (scalar-default)))
   (grid:copy-to (matrix-product-triangular m1 m2 s1))))

(generate-all-array-tests inverse-matrix-product 
			  #+fsbv :float-complex #-fsbv :float
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3)))
       (s1 (scalar-default)))
   (grid:copy-to (inverse-matrix-product m1 m2 s1))))

(generate-all-array-tests matrix-product-symmetric :float
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3)))
       (m3 (array-default '(3 3)))
       (s1 (scalar-default))
       (s2 (scalar-default)))
   (grid:copy-to (matrix-product-symmetric m1 m2 m3 s1 s2))))

#+fsbv
(generate-all-array-tests matrix-product-hermitian :complex
 (let ((m1 (array-default '(3 3)))
       (m2 (array-default '(3 3)))
       (m3 (array-default '(3 3)))
       (s1 (scalar-default))
       (s2 (scalar-default)))
   (grid:copy-to (matrix-product-hermitian m1 m2 m3 s1 s2))))
