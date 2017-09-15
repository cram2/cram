;; Define the structures for solvers
;; Liam Healy 2009-06-06 16:46:38EDT solver-struct.lisp
;; Time-stamp: <2016-08-07 19:37:54EDT solver-struct.lisp>
;;
;; Copyright 2009, 2010, 2016 Liam M. Healy
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

(include "gsl/gsl_multifit_nlin.h")

;; The definition of a solver instance and state
;; for nonlinear least squares fitting in GSL.

(cstruct gsl-fdffit-solver "gsl_multifit_fdfsolver"
  (f "f" :type :pointer)
  #-gsl2 (jacobian "J" :type :pointer)
  (dx "dx" :type :pointer))

(include "gsl/gsl_multiroots.h")

(cstruct gsl-multiroot-fsolver "gsl_multiroot_fsolver"
  (x "x" :type :pointer)
  (f "f" :type :pointer)
  (dx "dx" :type :pointer))
