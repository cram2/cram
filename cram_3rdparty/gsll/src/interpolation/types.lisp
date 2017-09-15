;; Interpolation types
;; Liam Healy, Sun Nov  4 2007 - 17:41
;; Time-stamp: <2009-12-27 09:52:12EST types.lisp>
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

(defmpar +linear-interpolation+ "gsl_interp_linear"
  ;; FDL
  "Linear interpolation.  This interpolation method does not require any
   additional memory.")

(defmpar +polynomial-interpolation+ "gsl_interp_polynomial"
  ;; FDL
  "Polynomial interpolation.  This method should only be used for
   interpolating small numbers of points because polynomial interpolation
   introduces large oscillations, even for well-behaved datasets.  The
   number of terms in the interpolating polynomial is equal to the number
   of points.")

(defmpar +cubic-spline-interpolation+ "gsl_interp_cspline"
  ;; FDL
  "Cubic spline with natural boundary conditions.  The resulting curve is
   piecewise cubic on each interval, with matching first and second
   derivatives at the supplied data-points.  The second derivative is
   chosen to be zero at the first point and last point.")

(defmpar +periodic-cubic-spline-interpolation+ "gsl_interp_cspline_periodic"
  ;; FDL
  "Cubic spline with periodic boundary conditions.  The resulting curve
   is piecewise cubic on each interval, with matching first and second
   derivatives at the supplied data-points.  The derivatives at the first
   and last points are also matched.  Note that the last point in the
   data must have the same y-value as the first point, otherwise the
   resulting periodic interpolation will have a discontinuity at the
   boundary.")

(defmpar +akima-interpolation+ "gsl_interp_akima"
  ;; FDL
  "Non-rounded Akima spline with natural boundary conditions.  This method
   uses the non-rounded corner algorithm of Wodicka.")

(defmpar +periodic-akima-interpolation+ "gsl_interp_akima_periodic"
  ;; FDL
  "Non-rounded Akima spline with periodic boundary conditions.  This method
   uses the non-rounded corner algorithm of Wodicka.")

(defmfun name ((interpolation interpolation))
  "gsl_interp_name"
  ((interpolation :pointer))
  :definition :method
  :c-return :string
  :documentation			; FDL
  "The name of the interpolation type.")

(export 'minimum-size)
(defgeneric minimum-size (object)
  (:documentation			; FDL
   "The minimum number of points required by the
   interpolation.  For example, Akima spline interpolation
   requires a minimum of 5 points."))

(defmfun minimum-size ((object interpolation))
  "gsl_interp_min_size"
  ((object :pointer))
  :definition :method
  :c-return :uint)

(defmfun name ((object spline))
  "gsl_spline_name"
  ((object :pointer))
  :definition :method
  :c-return :string
  :documentation			; FDL
  "The name of the interpolation type.")

(defmfun minimum-size ((object spline))
  "gsl_spline_min_size"
  ((object :pointer))
  :definition :method
  :c-return :uint)
