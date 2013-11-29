;; Interpolation allocation, initialization, and freeing.
;; Liam Healy, Sun Nov  4 2007 - 17:24
;; Time-stamp: <2010-06-27 18:13:56EDT interpolation.lisp>
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
;;; /usr/include/gsl/gsl_spline.h

;;; A spline is an interpolation that also stores the arrays xa and ya,
;;; so they need not be supplied on each call.
(defmobject interpolation "gsl_interp"
  ((type :pointer) (size sizet))
  "interpolation"
  :documentation			; FDL
  "Make an interpolation object of type for size data-points,
   and optionally initialize the interpolation object interp for the
  data (xa,ya) where xa and ya are vectors.  The interpolation object does not save
  the data arrays xa and ya and only stores the static state
  computed from the data.  The xa data array is always assumed to be
  strictly ordered; the behavior for other arrangements is not defined."
  :initialize-suffix "init"
  :initialize-args
  (((foreign-pointer xa) :pointer) ((foreign-pointer ya) :pointer) ((dim0 xa) sizet))
  :arglists-function
  (lambda (set)
    `((type &optional xa-or-size (ya nil ,set))
      (:type type :size (if ,set (dim0 ya) xa-or-size))
      (:xa xa-or-size :ya ya))))

(defmobject spline "gsl_spline"
  ((type :pointer) (size sizet))
  "spline"
  :documentation			; FDL
  "Make an interpolation object of type for size data-points."
  :initialize-suffix "init"
  :initialize-args
  (((foreign-pointer xa) :pointer) ((foreign-pointer ya) :pointer) ((dim0 xa) sizet))
  :inputs (xa ya)
  :arglists-function
  (lambda (set)
    `((type &optional xa-or-size (ya nil ,set))
      (:type type :size (if ,set (dim0 ya) xa-or-size))
      (:xa xa-or-size :ya ya))))
