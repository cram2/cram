;; CFFI-Grovel definitions for unix systems.
;; Liam Healy
;; Time-stamp: <2010-05-23 11:38:22EDT sf-result.lisp>
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

(include "gsl/gsl_sf_result.h")

;;; Results from special functions with value and error estimate.
;;; file:///usr/share/doc/gsl-ref-html/gsl-ref_7.html#SEC61
(cstruct sf-result "gsl_sf_result"
  (val "val" :type :double)
  (err "err" :type :double))

;;; Results from special functions with value, error estimate
;;; and a scaling exponent e10, such that the value is val*10^e10.
;;; file:///usr/share/doc/gsl-ref-html/gsl-ref_7.html#SEC61
(cstruct sf-result-e10 "gsl_sf_result_e10"
  (val "val" :type :double)
  (err "err" :type :double)
  (e10 "e10" :type :int))
