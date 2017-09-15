;; Define generic operators for GSL functions
;; Liam Healy 2011-01-01 09:51:47EST linear-algebra.lisp
;; Time-stamp: <2015-11-22 12:17:11EST linear-algebra.lisp>

;; Copyright 2011, 2013, 2015 Liam M. Healy
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

(in-package :antik)

(export '(invert-matrix determinant solve-linear))

;;;;****************************************************************************
;;;; Basic arithmetic
;;;;****************************************************************************

;;; Take care to copy arrays where the GSL/BLAS routines will
;;; overwrite an input array; matrix-product does not overwrite, but
;;; scale does.

(defmethod *i ((a grid:foreign-array) (b grid:foreign-array))
  (gsl:matrix-product a b))

(defmethod *i ((a grid:foreign-array) (b number))
  (gsl:elt* (grid:copy a) (coerce b (grid:element-type a))))

(defmethod *i ((a number) (b grid:foreign-array))
  (gsl:elt* (grid:copy b) (coerce a (grid:element-type b))))

(defmethod /i ((a grid:foreign-array) (b number))
  (gsl:elt* (grid:copy a)
	    (cl:/ (coerce b (grid:element-type a)))))

(defmethod +i ((a grid:foreign-array) (b grid:foreign-array))
  (gsl:elt+ (grid:copy a) b))

(defmethod +i ((a grid:foreign-array) (b number))
  (gsl:elt+ (grid:copy a)
	    (coerce b (grid:element-type a))))

(defmethod +i ((a number) (b grid:foreign-array))
  (gsl:elt+ (grid:copy b)
	    (coerce a (grid:element-type b))))

(defmethod -i ((a grid:foreign-array) (b grid:foreign-array))
  (gsl:elt- (grid:copy a) b))

(defmethod -i ((a grid:foreign-array) (b number))
  (gsl:elt+ (grid:copy a) (cl:- b)))

(defmethod -i ((a number) (b grid:foreign-array))
  (gsl:elt+ (- b) a))

(defmethod expt ((a grid:matrix) (b integer))
  (cond ((= 1 b) a)
	((plusp b)		   ; use addition-chain exponentiation
	 (* a (expt a (1- b))))
	((minusp b)
	 (expt (invert-matrix a) (- b)))
	;; zero exponent; need to figure out non-square matrix
	(t (grid:identity-matrix (grid:dim0 a)))))

;;;;****************************************************************************
;;;; Linear algebra using LU decomposition
;;;;****************************************************************************

;;; Eventually it would be nice to be able to specify the method, instead of only using LU.
;;; These assume that the supplied arrays are of type grid:foreign-array.

(defun invert-matrix (matrix)
  "Invert the matrix; return the inverse and determinant."
  (multiple-value-bind (LU permutation signum)
      (gsl:LU-decomposition (grid:copy matrix))
    (values
      (gsl:LU-invert LU permutation)
      (gsl:LU-determinant LU signum))))

(defun determinant (matrix)
  "Find the determinant of the matrix."
  (multiple-value-bind (LU permutation signum)
      (gsl:LU-decomposition (grid:copy matrix))
    (declare (ignore permutation))
    (gsl:LU-determinant LU signum)))

(defun solve-linear (matrix vector)
  "Solve the linear algebra equation."
  (multiple-value-bind (upper permutation signum)
      (gsl:LU-decomposition (grid:copy matrix))
    (declare (ignore signum))
    (let ((initial-solution (gsl:LU-solve upper vector permutation T)))
      (gsl:LU-refine matrix upper permutation vector initial-solution))))

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

#|
;;; 
(determinant gsl::*vander4*)
12.000000000000014

(* (invert-matrix gsl::*vander4*) gsl::*vander4*)
;;; gives something close to the identity

;;; These are direct tests of matrix inversion
(save-test
 lu
 (grid:copy
  (invert-matrix
   (grid:make-foreign-array 
    'double-float
    :initial-contents '((1.0d0 2.0d0) (3.0d0 4.0d0))))))

  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-1.9999999999999998d0 1.0d0)
	(1.4999999999999998d0 -0.49999999999999994d0)))
   (MULTIPLE-VALUE-LIST
    (GRID:COPY
     (INVERT-MATRIX
      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS '(2 2)
			       :INITIAL-CONTENTS
			       '((1.0d0 2.0d0) (3.0d0 4.0d0)))))))
|#

