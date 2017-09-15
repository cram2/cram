;; Householder Transformations
;; Liam Healy, Wed May 10 2006 - 10:03
;; Time-stamp: <2011-04-23 17:14:09EDT householder.lisp>
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

;;; For householder-transform, it would be nice to be able to pick the
;;; order of arguments returned (reverse).

(in-package :gsl)

;;;;****************************************************************************
;;;; Householder transformations
;;;;****************************************************************************

;;; FDL
;;; A Householder transformation is a rank-1 modification of the identity
;;; matrix which can be used to zero out selected elements of a vector.  A
;;; Householder matrix P takes the form,
;;; P = I - tau v v^T
;;; where v is a vector (called the Householder vector) and
;;; tau = 2/(v^T v).  The functions described in this section use the
;;; rank-1 structure of the Householder matrix to create and apply
;;; Householder transformations efficiently.

(defmfun householder-transform (v)
  "gsl_linalg_householder_transform"
  (((mpointer v) :pointer))
  :inputs (v)
  :outputs (v)
  :c-return (ret :double)
  :return (v ret)
  :documentation			; FDL
  "Prepare a Householder transformation P = I - tau v v^T
  which can be used to zero all the elements of the input vector except
  the first.  Returned values are the transformation, which is stored
  in the vector v, and the scalar tau.")

(defmfun householder-HM (tau v A)
  "gsl_linalg_householder_hm"
  ((tau :double) ((mpointer v) :pointer) ((mpointer A) :pointer))
  :inputs (v A)
  :outputs (A)
  :documentation			; FDL
  "Apply the Householder matrix P defined by the
  scalar tau and the vector v to the left-hand side of the
  matrix A.  On output the result P A is stored in A.")

(defmfun householder-MH (tau v A)
  "gsl_linalg_householder_mh"
  ((tau :double) ((mpointer v) :pointer) ((mpointer A) :pointer))
  :inputs (v A)
  :outputs (A)
  :documentation			; FDL
  "Apply the Householder matrix P defined by the
  scalar tau and the vector v to the right-hand side of the
  matrix A.  On output the result A P is stored in A.")

(defmfun householder-Hv (tau v w)
  "gsl_linalg_householder_hv"
  ((tau :double) ((mpointer v) :pointer) ((mpointer w) :pointer))
  :inputs (v w)
  :outputs (w)
  :documentation			; FDL
  "Apply the Householder transformation P defined by
  the scalar tau and the vector v to the vector w.  On
  output the result P w is stored in w.")

;;;;****************************************************************************
;;;; Householder solver for linear systems
;;;;****************************************************************************

(defmfun householder-solve
    (A b &optional x-spec
       &aux
       (x (grid:ensure-foreign-array x-spec (grid:dimensions b))))
  ("gsl_linalg_HH_svx" "gsl_linalg_HH_solve")
  ((((mpointer A) :pointer) ((mpointer b) :pointer))
   (((mpointer A) :pointer) ((mpointer b) :pointer)
    ((mpointer x) :pointer)))
  :inputs (A b)
  :outputs (x)
  :return ((or x b))
  :documentation			; FDL
  "Solve the system A x = b directly using Householder
   transformations. If x-spec is NIL (default), the solution will
   replace b.  If x-spec is T, then an array will be created and the
   solution returned in it.  If x-spec is a grid:foreign-array, the solution will
   be returned in it.  If x-spec is non-NIL, on output the solution is
   stored in x and b is not modified.  The matrix A is destroyed by
   the Householder transformations.  The solution is returned from the
   function call.")

;;;;****************************************************************************
;;;; Examples and unit test, from linalg/test.c
;;;;****************************************************************************

(defun test-hh-solve-dim (matrix)
  "Solve the linear equation using Householder with the supplied
   matrix and a right-hand side vector which is the reciprocal of one
   more than the index."
  (householder-solve (copy matrix) (create-rhs-vector (dim0 matrix)) T))

(save-test householder
 (test-hh-solve-dim *hilb2*)
 (test-hh-solve-dim *hilb3*)
 (test-hh-solve-dim *hilb4*)
 (test-hh-solve-dim *hilb12*)
 (test-hh-solve-dim *vander2*)
 (test-hh-solve-dim *vander3*)
 (test-hh-solve-dim *vander4*)
 (test-hh-solve-dim *vander12*))
