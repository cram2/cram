;; Constants specifying limits of floating point calculations in hardware 
;; Liam Healy
;; Time-stamp: <2010-06-24 09:54:03EDT machine.lisp>
;;
;; Copyright 2010 Liam M. Healy
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

(include "gsl/gsl_machine.h")

(constant (+dbl-epsilon+ "GSL_DBL_EPSILON") :type double-float)
(constant (+sqrt-dbl-epsilon+ "GSL_SQRT_DBL_EPSILON") :type double-float)
(constant (+log-dbl-max+ "GSL_LOG_DBL_MAX") :type double-float)
