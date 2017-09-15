;; BLAS level 2, Matrix-vector operations
;; Liam Healy, Wed Apr 26 2006 - 21:08
;; Time-stamp: <2013-12-25 12:08:15EST blas2.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2010, 2011, 2013 Liam M. Healy
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
;;;; Functions
;;;;****************************************************************************

(defun matrix-product-dimensions (a b &key (transa :notrans) (transb :notrans))
  (let ((dima (funcall
	       (ecase transa
		 (:notrans #'first)
		 (:trans #'second))
	       (grid:dimensions a))))
    (if (typep b 'grid:matrix)
	(list dima
	      (funcall
	       (ecase transb
		 (:notrans #'second)
		 (:trans #'first))
	       (grid:dimensions b)))
	dima)))

(defmfun matrix-product
    ((A grid:matrix) (x vector)
     &optional
     y
     (alpha 1) (beta 1) (TransA :notrans) TransB
     &aux
     (yarr
      (grid:ensure-foreign-array
       y (matrix-product-dimensions A x :transa transa) element-type 0)))
  ("gsl_blas_" :type "gemv")
  ((transa cblas-transpose) (alpha :element-c-type) ((mpointer A) :pointer)
   ((mpointer x) :pointer) (beta :element-c-type) ((mpointer yarr) :pointer))
  :definition :generic
  :element-types #+fsbv :float-complex #-fsbv :float
  :inputs (A x)
  :outputs (yarr)
  :documentation			; FDL
  "If the second and third arguments are vectors, compute
   the matrix-vector product and sum
    y = alpha op(A) x + beta y, where op(A) = A, A^T, A^H
  for TransA = :notrans, :trans, :conjtrans.
  If the second and third arguments are matrices, compute
  the matrix-matrix product and sum C = alpha
  op(A) op(B) + beta C where op(A) = A, A^T, A^H for TransA =
  :notrans, :trans, :conjtrans and similarly for the
  parameter TransB.")

(defmfun matrix-product-triangular
    ((A grid:matrix) (x vector)
     &optional (alpha 1) (uplo :upper) (TransA :notrans)
     (diag :nonunit) (side :left))
  ("gsl_blas_" :type "trmv")
  ((uplo cblas-uplo) (TransA cblas-transpose) (diag cblas-diag)
   ((mpointer A) :pointer) ((mpointer x) :pointer))
  :definition :generic
  :element-types :float-complex
  :inputs (A x)
  :outputs (x)
  :documentation			; FDL
  "If the second argument is a vector, compute
   the matrix-vector product x = op(A) x
   for the triangular matrix A, where op(A) = A, A^T, A^H for
   TransA = :NoTrans, :Trans, :ConjTrans. When Uplo
   is :Upper then the upper triangle of A is used, and when
   Uplo is :Lower then the lower triangle of A is used. If
   Diag is :NonUnit then the diagonal of the matrix is used,
   but if Diag is :Unit then the diagonal elements of the
   matrix A are taken as unity and are not referenced.
   If the second argument is a matrix, compute
   the matrix-matrix product B = alpha op(A) B
   if Side is :Left and B = alpha B op(A) if Side is
   :Right. The matrix A is triangular and op(A) = A, A^T, A^H
   for TransA = :NoTrans, :Trans, :ConjTrans When Uplo
   is :Upper then the upper triangle of A is used, and when
   Uplo is :Lower then the lower triangle of A is used. If
   Diag is :NonUnit then the diagonal of A is used, but if
   Diag is :Unit then the diagonal elements of the matrix A
   are taken as unity and are not referenced.")

(defmfun inverse-matrix-product
    ((A grid:matrix) (x vector)
     &optional (alpha 1) (uplo :upper) (TransA :notrans)
     (diag :nonunit) (side :left))
  ("gsl_blas_" :type "trsv")
  ((uplo cblas-uplo) (TransA cblas-transpose) (diag cblas-diag)
   ((mpointer A) :pointer) ((mpointer x) :pointer))
  :definition :generic
  :element-types :float-complex
  :inputs (A x)
  :outputs (x)
  :documentation			; FDL
   "If the second argument is a vector, compute
   inv(op(A)) x for x, where op(A) = A, A^T, A^H for
   TransA = :NoTrans, :Trans, :ConjTrans. When Uplo is
   :Upper then the upper triangle of A is used, and when Uplo is
   :Lower then the lower triangle of A is used. If Diag is
   :NonUnit then the diagonal of the matrix is used, but if Diag
   is :Unit then the diagonal elements of the matrix A are taken
   as unity and are not referenced.
   If the second argument is a matrix, compute
   the inverse-matrix matrix product B = alpha op(inv(A))B if
   Side is :Left and B = alpha B op(inv(A)) if Side is
   :Right. The matrix A is triangular and op(A) = A, A^T, A^H
   for TransA = :NoTrans, :Trans, :ConjTrans When
   Uplo is :Upper then the upper triangle of A is used, and
   when Uplo is :Lower then the lower triangle of A is
   used. If Diag is :NonUnit then the diagonal of A is used,
   but if Diag is :Unit then the diagonal elements of the
   matrix A are taken as unity and are not referenced.")

(defmfun matrix-product-symmetric
    ((A grid:matrix) (x vector)
     &optional y
     (alpha 1) (beta 1) (uplo :upper) (side :left)
     &aux
     (yarr
      (or y
	  (grid:make-foreign-array element-type :dimensions (matrix-product-dimensions A x)
		       :initial-element 0))))
  ("gsl_blas_" :type "symv")
  ((uplo cblas-uplo) (alpha :element-c-type) ((mpointer A) :pointer)
   ((mpointer x) :pointer) (beta :element-c-type) ((mpointer yarr) :pointer))
  :definition :generic
  :element-types :float
  :inputs (A x yarr)
  :outputs (yarr)
  :documentation			; FDL
  "If the second and third arguments are vectors, compute
  the matrix-vector product and sum y = alpha A
  x + beta y for the symmetric matrix A. Since the matrix A is
  symmetric only its upper half or lower half need to be
  stored. When Uplo is :Upper then the upper triangle and
  diagonal of A are used, and when Uplo is :Lower then the
  lower triangle and diagonal of A are used.
  If the second and third arguments are matrices, compute
  the matrix-matrix product and sum C = alpha A
  B + beta C for Side is :Left and C = alpha B A + beta C
  for Side is :Right, where the matrix A is symmetric. When
  Uplo is :Upper then the upper triangle and diagonal of A
  are used, and when Uplo is :Lower then the lower triangle
  and diagonal of A are used.")

#+fsbv
(defmfun matrix-product-hermitian
    ((A grid:matrix) (x vector)
     &optional
     (y (grid:make-foreign-array element-type :dimensions (matrix-product-dimensions A x)
		     :initial-element 0))
     (alpha 1) (beta 1) (uplo :upper) (side :left))
  ("gsl_blas_" :type "hemv")
  ((uplo cblas-uplo) (alpha :element-c-type) ((mpointer A) :pointer)
   ((mpointer x) :pointer) (beta :element-c-type) ((mpointer y) :pointer))
  :definition :generic
  :element-types :complex
  :inputs (A x y)
  :outputs (y)
  :documentation			; FDL
  "If the second and third arguments are vectors, compute the
  matrix-vector product and sum y = alpha A x + beta y for the
  hermitian matrix A. Since the matrix A is hermitian only its upper
  half or lower half need to be stored. When Uplo is :upper then
  the upper triangle and diagonal of A are used, and when Uplo is
  :lower then the lower triangle and diagonal of A are used. The
  imaginary elements of the diagonal are automatically assumed to be
  zero and are not referenced.  If the second and third arguments are
  matrices, compute the matrix-matrix product and sum C = alpha A B +
  beta C if Side is :left and C = \alpha B A + \beta C if Side
  is :right, where the matrix A is hermitian. When Uplo is
  :upper then the upper triangle and diagonal of A are used, and
  when Uplo is :lower then the lower triangle and diagonal of A
  are used. The imaginary elements of the diagonal are automatically
  set to zero.")

(defmfun rank-1-update (alpha (x vector) (y vector) (A grid:matrix))
  ("gsl_blas_" :type "ger" :suffix)
  ((alpha :element-c-type) ((mpointer x) :pointer)
   ((mpointer y) :pointer) ((mpointer A) :pointer))
  :definition :generic
  :element-types #+fsbv :float-complex #-fsbv :float
  :inputs (x y A)
  :outputs (A)
  :documentation			; FDL
   "The rank-1 update A = alpha x y^T + A of the matrix A.")

#+fsbv
(defmfun conjugate-rank-1-update (alpha (x vector) (y vector) (A grid:matrix))
  ("gsl_blas_" :type "gerc")
  ((alpha :element-c-type) ((mpointer x) :pointer)
   ((mpointer y) :pointer) ((mpointer A) :pointer))
  :definition :generic
  :element-types :complex
  :inputs (x y A)
  :outputs (A)
  :documentation			; FDL
  "The conjugate rank-1 update A = alpha x y^H + A of the matrix A.")

(defmfun symmetric-rank-1-update
    ((x vector) (A grid:matrix)
     &optional (alpha 1) (beta 1) (uplo :upper) (trans :notrans))
  ("gsl_blas_" :type "syr")
  ((uplo cblas-uplo) (alpha :element-c-type)
   ((mpointer x) :pointer) ((mpointer A) :pointer))
  :definition :generic
  :element-types :float
  :inputs (x A)
  :outputs (A)
  :documentation			; FDL
  "If the first argument is a vector,
  the symmetric rank-1 update A = \alpha x x^T + A of the symmetric
  matrix A. Since the matrix A is symmetric only its upper half or
  lower half need to be stored. When Uplo is :Upper then the upper
  triangle and diagonal of A are used, and when Uplo is :Lower then
  the lower triangle and diagonal of A are used.  If the first
  argument is a matrix, a rank-k update of the symmetric matrix C, C =
  \alpha A A^T + \beta C when Trans is CblasNoTrans and C = \alpha A^T
  A + \beta C when Trans is CblasTrans. Since the matrix C is
  symmetric only its upper half or lower half need to be stored. When
  Uplo is CblasUpper then the upper triangle and diagonal of C are
  used, and when Uplo is CblasLower then the lower triangle and
  diagonal of C are used.")

#+fsbv
(defmfun hermitian-rank-1-update
    ((x vector) (A grid:matrix)
     &optional (alpha 1) (beta 1) (uplo :upper) (trans :notrans))
  ("gsl_blas_" :type "her")
  ((uplo cblas-uplo) (alpha :element-c-type)
   ((mpointer x) :pointer) ((mpointer A) :pointer))
  :definition :generic
  :element-types :complex
  :inputs (x A)
  :outputs (A)
  :documentation			; FDL
  "If the first argument is a vector,
  compute the hermitian rank-1 update A = alpha x x^H + A of the
  hermitian matrix A. Since the matrix A is hermitian only its upper
  half or lower half need to be stored. When Uplo is :upper then the
  upper triangle and diagonal of A are used, and when Uplo is
  :lower then the lower triangle and diagonal of A are used. The
  imaginary elements of the diagonal are automatically set to zero.
  If the first argument is a matrix, compute a rank-k update of the
  hermitian matrix C, C = \alpha A A^H + \beta C when Trans is
  :notrans and C = \alpha A^H A + \beta C when Trans is
  :trans. Since the matrix C is hermitian only its upper half or
  lower half need to be stored. When Uplo is :upper then the upper
  triangle and diagonal of C are used, and when Uplo is :lower
  then the lower triangle and diagonal of C are used. The imaginary
  elements of the diagonal are automatically set to zero.")

(defmfun symmetric-rank-2-update
    ((x vector) (y vector) (A grid:matrix)
     &optional (alpha 1) (beta 1) (uplo :upper) (trans :notrans))
  ("gsl_blas_" :type "syr2")
  ((uplo cblas-uplo) (alpha :element-c-type)
   ((mpointer x) :pointer)  ((mpointer y) :pointer)
   ((mpointer A) :pointer))
  :definition :generic
  :element-types :float
  :inputs (x y A)
  :outputs (A)
  :documentation			; FDL
  "If the first two arguments are vectors, compute
  the symmetric rank-2 update A = alpha x y^T + alpha y x^T + A of the
  symmetric matrix A. Since the matrix A is symmetric only its upper
  half or lower half need to be stored. When Uplo is :upper then the
  upper triangle and diagonal of A are used, and when Uplo is :lower
  then the lower triangle and diagonal of A are used.  If the first
  two arguments are matrices, compute a rank-2k update of the
  symmetric matrix C, C = \alpha A B^T + \alpha B A^T + \beta C when
  Trans is :notrans and C = \alpha A^T B + \alpha B^T A + \beta C
  when Trans is :trans. Since the matrix C is symmetric only its
  upper half or lower half need to be stored. When Uplo is :upper
  then the upper triangle and diagonal of C are used, and when Uplo is
  :lower then the lower triangle and diagonal of C are used.")

#+fsbv
(defmfun hermitian-rank-2-update
    ((x vector) (y vector) (A grid:matrix)
     &optional (alpha 1) (beta 1) (uplo :upper) (trans :notrans))
  ("gsl_blas_" :type "her2")
  ((uplo cblas-uplo) (alpha :element-c-type)
   ((mpointer x) :pointer) ((mpointer A) :pointer))
  :definition :generic
  :element-types :complex
  :inputs (x A)
  :outputs (A)
  :documentation			; FDL
  "If the first two arguments are vectors, compute the
  hermitian rank-2 update A = alpha x y^H + alpha^* y x^H A of
  the hermitian matrix A. Since the matrix A is hermitian only its
  upper half or lower half need to be stored. When uplo is :upper
  then the upper triangle and diagonal of A are used, and when uplo is
  :lower then the lower triangle and diagonal of A are used. The
  imaginary elements of the diagonal are automatically set to zero.
  If the first two arguments are matrices, compute a rank-2k update of
  the hermitian matrix C, C = \alpha A B^H + \alpha^* B A^H + \beta C
  when Trans is :notrans and C = \alpha A^H B + \alpha^* B^H A +
  \beta C when Trans is :conjtrans. Since the matrix C is
  hermitian only its upper half or lower half need to be stored. When
  Uplo is :upper then the upper triangle and diagonal of C are
  used, and when Uplo is :lower then the lower triangle and
  diagonal of C are used. The imaginary elements of the diagonal are
  automatically set to zero.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(generate-all-array-tests matrix-product #+fsbv :float-complex #-fsbv :float
 (let ((m1 (array-default '(3 3)))
       (v1 (array-default 3))
       (v2 (array-default 3))
       (s1 (scalar-default))
       (s2 (scalar-default)))
   (grid:copy-to (matrix-product m1 v1 v2 s1 s2))))

(generate-all-array-tests matrix-product-triangular
			  #+fsbv :float-complex #-fsbv :float
 (let ((m1 (array-default '(3 3)))
       (v1 (array-default 3))
       (s1 (scalar-default)))
   (grid:copy-to (matrix-product-triangular m1 v1 s1))))

(generate-all-array-tests inverse-matrix-product
			  #+fsbv :float-complex #-fsbv :float
 (let ((m1 (array-default '(3 3)))
       (v1 (array-default 3))
       (s1 (scalar-default)))
   (grid:copy-to (inverse-matrix-product m1 v1 s1))))

(generate-all-array-tests matrix-product-symmetric :float
 (let ((m1 (array-default '(3 3)))
	(v1 (array-default 3))
	(v3 (array-default 3))
	(s1 (scalar-default))
	(s2 (scalar-default)))
   (grid:copy-to (matrix-product-symmetric m1 v1 v3 s1 s2))))

#+fsbv
(generate-all-array-tests matrix-product-hermitian :complex
 (let ((m1 (array-default '(3 3)))
       (v1 (array-default 3))
       (v2 (array-default 3))
       (s1 (scalar-default))
       (s2 (scalar-default)))
   (grid:copy-to (matrix-product-hermitian m1 v1 v2 s1 s2))))

(generate-all-array-tests rank-1-update :float-complex
 (let ((m1 (array-default '(3 3)))
	(v1 (array-default 3))
	(v2 (array-default 3))
	(s1 (scalar-default)))
   (grid:copy-to (rank-1-update s1 v1 v2 m1))))

