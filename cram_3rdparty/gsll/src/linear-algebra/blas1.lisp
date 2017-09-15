;; BLAS level 1, Vector operations
;; Liam Healy, Wed Apr 26 2006 - 15:23
;; Time-stamp: <2012-01-13 12:01:25EST blas1.lisp>
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

;;; /usr/include/gsl/gsl_blas.h

(defmfun grid:inner ((vec1 vector) (vec2 vector))
  ("gsl_blas_" :type "dot" :suffix)
  (((mpointer vec1) :pointer) ((mpointer vec2) :pointer)
   (result (:pointer :element-c-type)))
  :definition :methods
  :element-types :float-complex
  :inputs (vec1 vec2)
  :documentation			; FDL
  "Dot, or inner, product between vectors.")

;;; gsl_blas_sdsdot doesn't make much sense, but here it is.
(defmfun sdot (result alpha vec1 vec2)
  "gsl_blas_sdsdot"
  ((alpha :float) ((mpointer vec1) :pointer) ((mpointer vec2) :pointer)
   (result :pointer))
  :inputs (vec1 vec2)
  :outputs (result)
  :documentation			; FDL
  "Sum of a scalar and a dot product for single-floats.")

;;; Not porting gsl_blas_dsdot, stupid.

(defmfun cdot ((x vector) (y vector))
  ("gsl_blas_" :type "dotc")
  (((mpointer x) :pointer) ((mpointer y) :pointer)
   (result (:pointer :element-c-type)))
  :definition :generic
  :element-types :complex
  :inputs (x y)
  :documentation			; FDL
  "The complex conjugate scalar product x^H y for the vectors.")

(defmfun euclidean-norm ((vec vector))
  ("gsl_blas_" :component-float-type :type "nrm2")
  (((mpointer vec) :pointer))
  :definition :generic
  :element-types :float-complex
  :inputs (vec)
  :c-return :component-float-type
  :documentation			; FDL
  "The Euclidean norm ||x||_2 = \sqrt {\sum x_i^2} of the vector x.")

(defmfun absolute-sum ((x vector))
  ("gsl_blas_" :component-float-type :type "asum")
  (((mpointer x) :pointer))
  :definition :generic
  :element-types :float-complex
  :c-return :component-float-type
  :inputs (x)
  :documentation			; FDL
  "The absolute sum \sum |x_i| of the elements of the vector x.")

(defmfun index-max ((vec vector))
  ("gsl_blas_i" :type "amax")
  (((mpointer vec) :pointer))
  :definition :generic
  :element-types :float-complex
  :c-return :sizet
  :inputs (vec)
  :documentation			; FDL
  "The index of the largest element of the vector
   x. The largest element is determined by its absolute magnitude for
   real vectors and by the sum of the magnitudes of the real and
   imaginary parts |\Re(x_i)| + |\Im(x_i)| for complex vectors. If the
   largest value occurs several times then the index of the first
   occurrence is returned.")

(defmfun blas-swap ((vec1 vector) (vec2 vector))
  ("gsl_blas_" :type "swap")
  (((mpointer vec1) :pointer) ((mpointer vec2) :pointer))
  :definition :generic
  :element-types :float-complex
  :inputs (vec1 vec2)
  :outputs (vec1 vec2)
  :documentation			; FDL
  "Exchange the elements of the vectors.")

(defmfun blas-copy ((x vector) (y vector))
  ("gsl_blas_" :type "copy")
  (((mpointer x) :pointer) ((mpointer y) :pointer))
  :definition :generic
  :element-types :float-complex
  :inputs (x)
  :outputs (y)
  :documentation			; FDL
  "Copy the elements of the vector x into the vector y.")

(defmfun axpy
    (alpha (x vector)
	   &optional
	   (y (grid:make-foreign-array element-type :dimensions (grid:dimensions x))))
  ;; This gets an error for complex types because you can't pass a
  ;; struct in CFFI yet.
  ("gsl_blas_" :type "axpy")
  ((alpha  :element-c-type) ((mpointer x) :pointer) ((mpointer y) :pointer))
  :definition :generic
  :element-types #+fsbv :float-complex #-fsbv :float
  :inputs (x y)
  :outputs (y)
  :documentation			; FDL
  "Compute the sum y = \alpha x + y for the vectors x and y.")

(defmfun scale ((alpha :element-type) (x vector))
  ;; Alpha is the same type as the elements of vector, so for complex
  ;; vectors it must be complex.
  ("gsl_blas_" :type "scal")
  ((alpha :element-c-type) ((mpointer x) :pointer))
  :definition :generic
  :element-types #+fsbv :float-complex #-fsbv :float
  :c-return :void
  :inputs (x)
  :outputs (x)
  :documentation			; FDL
  "Rescale the vector x by the multiplicative factor alpha.")

(defmfun scale ((alpha :component-float-type) (x vector))
  ;; Alpha is a float and the vector is complex
  ("gsl_blas_" :type :component-float-type "scal")
  ((alpha :component-float-type) ((mpointer x) :pointer))
  :definition :methods
  :element-types :complex
  :c-return :void
  :inputs (x)
  :outputs (x))

;;; The Givens rotations come in two forms, those that work on bare C
;;; arrays, and those that work on GSL vectors.  Ports of both are
;;; present.

(defmfun givens-rotation ((x vector) (y vector) (c vector) (s vector))
  ("gsl_blas_" :type "rotg")
  (((grid:foreign-pointer x) :pointer) ((grid:foreign-pointer y) :pointer)
   ((grid:foreign-pointer c) :pointer) ((grid:foreign-pointer s) :pointer))
  :definition :generic
  :element-types :float
  :inputs (x y c s)
  :outputs (x y)
  :documentation			; FDL
  "These functions compute a Givens rotation (c,s) to the vector (x,y),
          [  c  s ] [ x ] = [ r ]
          [ -s  c ] [ y ]   [ 0 ]
   The variables x and y are overwritten by the routine.")

(defmfun givens-rotation-m ((x vector) (y vector) (c vector) (s vector))
  ("gsl_blas_" :type "rot")
  (((mpointer x) :pointer) ((mpointer y) :pointer)
   ((mpointer c) :pointer) ((mpointer s) :pointer))
  :definition :generic
  :element-types :float
  :inputs (x y c s)
  :outputs (x y)
  :documentation			; FDL
  "These functions compute a Givens rotation (c,s) to the vector (x,y),
          [  c  s ] [ x ] = [ r ]
          [ -s  c ] [ y ]   [ 0 ]
   The variables x and y are overwritten by the routine.")

(defmfun modified-givens-rotation
    ((d1 vector) (d2 vector) (b1 vector) b2 (P vector))
  ("gsl_blas_" :type "rotmg")
  (((grid:foreign-pointer d1) :pointer) ((grid:foreign-pointer d2) :pointer)
   ((grid:foreign-pointer b1) :pointer) (b2 :element-c-type)
   ((grid:foreign-pointer P) :pointer))
  :definition :generic
  :element-types :float
  :inputs (d1 d2 b1 P)
  :outputs ()				; I have no idea
  :documentation
  "Not explained")

(defmfun modified-givens-rotation-m ((x vector) (y vector) (P vector))
  ("gsl_blas_" :type "rotm")
  (((mpointer x) :pointer) ((mpointer y) :pointer)
   ((mpointer P) :pointer))
  :definition :generic
  :element-types :float
  :inputs (x y P)
  :outputs (x y)			;?????
  :documentation	
  "Not explained")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(generate-all-array-tests dot :float-complex
 (let ((v1 (array-default 8))
       (v2 (array-default 8)))
   (grid:inner v1 v2)))

(generate-all-array-tests cdot :complex
 (let ((v1 (array-default 8))
       (v2 (array-default 8)))
   (cdot v1 v2)))

(generate-all-array-tests euclidean-norm :float-complex
 (let ((v1 (array-default 8)))
   (euclidean-norm v1)))

(generate-all-array-tests absolute-sum :float-complex
 (let ((v1 (array-default 8)))
   (absolute-sum v1)))

(generate-all-array-tests index-max :float-complex
 (let ((v1 (array-default 8)))
   (index-max v1)))

(generate-all-array-tests blas-swap :float-complex
 (let ((v1 (array-default 3))
	(v2 (array-default 3)))
   (blas-swap v2 v1)
   (list (grid:copy-to v1) (grid:copy-to v2))))

(generate-all-array-tests blas-copy :float-complex
 (let ((v1 (array-default 3))
	(v2 (array-default 3 t)))
   (blas-copy v1 v2)
   (grid:copy-to v2)))

(generate-all-array-tests axpy :float-complex
 (let ((v1 (array-default 8))
	(v2 (array-default 8))
	(scalar (scalar-default)))
   (grid:copy-to (axpy scalar v1 v2))))

(generate-all-array-tests scale #+fsbv :float-complex #-fsbv :float
 (let ((v1 (array-default 8))
	(scalar (scalar-default)))
   (grid:copy-to (scale scalar v1))))

(generate-all-array-tests scale :complex
 (let ((v1 (array-default 8))
       (scalar (scalar-default t)))
   (grid:copy-to (scale scalar v1))))

(generate-all-array-tests givens :float
 (let ((v1 (array-default 8))
       (v2 (array-default 8))
       (angles (array-default 8))
       (sines (array-default 8 t))
       (cosines (array-default 8 t)))
   (loop for i below 8 do
	(setf (grid:aref sines i) (sin (grid:aref angles i)))
	(setf (grid:aref cosines i) (cos (grid:aref angles i))))
   (givens-rotation v1 v2 cosines sines)
   (list (grid:copy-to v1) (grid:copy-to v2))))
