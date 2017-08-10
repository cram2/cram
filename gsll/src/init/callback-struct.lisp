;; GSL structures for holding functions
;; Liam Healy 2009-04-04 22:15:56EDT callback-struct.lisp
;; Time-stamp: <2012-01-13 12:01:26EST callback-struct.lisp>
;;
;; Copyright 2009, 2010 Liam M. Healy
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

#+linux
(define "_GNU_SOURCE")

;;; When installed through Mac Ports, GSL .h files will be found
;;; in /opt/local/include.
#+darwin
(cc-flags #.(gsl-config "--cflags"))

(include "gsl/gsl_math.h")

;;; Only the function
;;; also in /usr/include/gsl/gsl_ntuple.h as gsl_ntuple_select_fn and
;;; gsl_ntuple_value_fn.
(cstruct fnstruct "gsl_function"
  (function "function" :type :pointer)
  (parameters "params" :type :pointer))

;;; The function and its derivative(s)
(cstruct fnstruct-fdf "gsl_function_fdf"
  (function "f" :type :pointer)
  (df "df" :type :pointer)
  (fdf "fdf" :type :pointer)
  (parameters "params" :type :pointer))

(include "gsl/gsl_multiroots.h") 

;;; The function and a dimension
;;; also defined in /usr/include/gsl/gsl_monte.h
(cstruct fnstruct-dimension "gsl_multiroot_function"
  (function "f" :type :pointer)
  (dimension "n" :type :sizet)
  (parameters "params" :type :pointer))

;;; The function, dimension, and derivatives
(cstruct fnstruct-dimension-fdf "gsl_multiroot_function_fdf"
  (function "f" :type :pointer)
  (df "df" :type :pointer)
  (fdf "fdf" :type :pointer)
  (dimension "n" :type :sizet)
  (parameters "params" :type :pointer))

(include "gsl/gsl_multifit_nlin.h") 

;;; The function and two dimensions (for nonlinear fit).
(cstruct fnstruct-fit "gsl_multifit_function"
  (function "f" :type :pointer)
  (number-of-observations "n" :type :sizet)
  (number-of-parameters "p" :type :sizet)
  (parameters "params" :type :pointer))

;;; The function, two dimensions, and derivatives (for nonlinear fit)
(cstruct fnstruct-fit-fdf "gsl_multifit_function_fdf"
  (function "f" :type :pointer)
  (df "df" :type :pointer)
  (fdf "fdf" :type :pointer)
  (number-of-observations "n" :type :sizet)
  (number-of-parameters "p" :type :sizet)
  (parameters "params" :type :pointer))
