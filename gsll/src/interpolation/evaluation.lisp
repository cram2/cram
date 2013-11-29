;; Evaluation of interpolation functions.
;; Liam Healy Sun Nov  4 2007 - 18:40
;; Time-stamp: <2010-06-27 18:13:56EDT evaluation.lisp>
;;
;; Copyright 2007, 2008, 2009 Liam M. Healy
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

;;; /usr/include/gsl/gsl_interp.h

(defmfun evaluate
    ((interpolation interpolation) x
     &key xa ya (acceleration (make-acceleration)))
  "gsl_interp_eval"
  (((mpointer interpolation) :pointer)
   ((foreign-pointer xa) :pointer)
   ((foreign-pointer ya) :pointer)
   (x :double)
   ((mpointer acceleration) :pointer))
  :definition :method
  :inputs (xa ya)
  :c-return :double
  :documentation			; FDL
  "Find the interpolated value of y for a given
   point x, using the interpolation object interpolation, data arrays
   xa and ya and the accelerator acceleration.")

(export 'evaluate-derivative)
(defgeneric evaluate-derivative (object point &key)
  (:documentation			; FDL
   "Find the derivative of an interpolated function for a given point
   x, using the interpolation object interpolation, data arrays
   xa and ya and the accelerator acceleration."))

(defmfun evaluate-derivative
    ((interpolation interpolation) x
     &key xa ya (acceleration (make-acceleration)))
  "gsl_interp_eval_deriv"
  (((mpointer interpolation) :pointer)
   ((foreign-pointer xa) :pointer)
   ((foreign-pointer ya) :pointer)
   (x :double)
   ((mpointer acceleration) :pointer))
  :definition :method
  :inputs (xa ya)
  :c-return :double)

(export 'evaluate-second-derivative)
(defgeneric evaluate-second-derivative (object point &key)
  (:documentation			; FDL
   "Find the second derivative of an interpolated function for a given point
   x, using the interpolation object interpolation, data arrays
   xa and ya and the accelerator acceleration."))

(defmfun evaluate-second-derivative
    ((interpolation interpolation) x &key xa ya (acceleration (make-acceleration)))
  "gsl_interp_eval_deriv2"
  (((mpointer interpolation) :pointer)
   ((foreign-pointer xa) :pointer)
   ((foreign-pointer ya) :pointer)
   (x :double)
   ((mpointer acceleration) :pointer))
  :definition :method
  :inputs (xa ya)
  :c-return :double)

(export 'evaluate-integral)
(defgeneric evaluate-integral (object lower-limit upper-limit &key)
  (:documentation			; FDL
   "Find the numerical integral of an interpolated function over the
   range [low, high], using the interpolation object interpolation,
   data arrays xa and ya and the accelerator 'acceleration."))

(defmfun evaluate-integral
    ((interpolation interpolation) low high
     &key xa ya (acceleration (make-acceleration)))
  "gsl_interp_eval_integ"
  (((mpointer interpolation) :pointer)
   ((foreign-pointer xa) :pointer)
   ((foreign-pointer ya) :pointer)
   (low :double) (high :double) ((mpointer acceleration) :pointer))
  :definition :method
  :inputs (xa ya)
  :c-return :double)

;;; Spline
(defmfun evaluate ((spline spline) x &key (acceleration (make-acceleration)))
  "gsl_spline_eval"
  (((mpointer spline) :pointer) (x :double) ((mpointer acceleration) :pointer))
  :definition :method
  :c-return :double)

(defmfun evaluate-derivative
    ((spline spline) x &key (acceleration (make-acceleration)))
  "gsl_spline_eval_deriv"
  (((mpointer spline) :pointer) (x :double) ((mpointer acceleration) :pointer))
  :definition :method
  :c-return :double)

(defmfun evaluate-second-derivative
    ((spline spline) x &key (acceleration (make-acceleration)))
  "gsl_spline_eval_deriv2"
  (((mpointer spline) :pointer) (x :double) ((mpointer acceleration) :pointer))
  :definition :method
  :c-return :double)

(defmfun evaluate-integral
    ((spline spline) low high &key (acceleration (make-acceleration)))
  "gsl_spline_eval_integ"
  (((mpointer spline) :pointer) (low :double) (high :double)
   ((mpointer acceleration) :pointer))
  :definition :method
  :c-return :double)
