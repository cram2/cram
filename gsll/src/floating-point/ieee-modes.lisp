;; IEEE 754 Modes and masks
;; Liam Healy 2008-01-29 21:35:50EST ieee-modes.lisp
;; Time-stamp: <2009-12-27 09:46:01EST ieee-modes.lisp>
;;
;; Copyright 2008, 2009 Liam M. Healy
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

#+ieee-floating-point
(defmfun set-floating-point-modes (precision rounding exception-mask)
  "gsl_ieee_set_mode"
  (((cffi:foreign-enum-value 'ieee-precisions precision) :int)
   ((cffi:foreign-enum-value 'ieee-rounding rounding) :int)
   ((cffi:foreign-enum-value 'ieee-mask exception-mask) :int))
  :documentation
  "Set the IEEE 754 precision, rounding mode, and exception mask.")
