;; Gegenbauer polynomials
;; Liam Healy, Fri Apr 28 2006 - 20:40
;; Time-stamp: <2011-10-29 23:29:22EDT gegenbauer.lisp>
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

(defmfun gegenbauer-1 (lambda x)
  "gsl_sf_gegenpoly_1_e"
  ((lambda :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Gegenbauer polynomial C^{(\lambda)}_1(x)}.")

(defmfun gegenbauer-2 (lambda x)
  "gsl_sf_gegenpoly_2_e"
  ((lambda :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Gegenbauer polynomial C^{(\lambda)}_2(x)}.")

(defmfun gegenbauer-3 (lambda x)
  "gsl_sf_gegenpoly_3_e"
  ((lambda :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Gegenbauer polynomial C^{(\lambda)}_3(x)}.")

(defmfun gegenbauer (n lambda x)
  "gsl_sf_gegenpoly_n_e"
  ((n :int) (lambda :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Gegenbauer polynomial C^{(\lambda)}_n(x)} for a specific value of n,
  lambda, x subject to \lambda > -1/2, n >= 0.")

(defmfun gegenbauer-array
    (lambda x &optional (size-or-array *default-sf-array-size*)
	    &aux (array (vdf size-or-array)))
  "gsl_sf_gegenpoly_array"
  (((1- (dim0 array)) :int)
   (lambda :double) (x :double) ((grid:foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "Compute an array of Gegenbauer polynomials C^{(\lambda)}_n(X)}
   for n = 0, 1, 2, ..., length(array)-1}, subject to \lambda > -1/2.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test gegenbauer
  (gegenbauer-1 1.0d0 3.0d0)
  (gegenbauer-2 1.0d0 3.0d0)
  (gegenbauer-3 1.0d0 3.0d0)
  (gegenbauer 4 1.0d0 3.0d0)
  (grid:copy-to (gegenbauer-array 1.0d0 3.0d0 4)))
