;; Regression test EXPONENTIAL-POWER for GSLL, automatically generated
;;
;; Copyright 2009 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST EXPONENTIAL-POWER
  ;; From randist/test.c
  (lisp-unit::assert-true
   (testpdf (lambda (r) (exponential-power-pdf r 3.7d0 0.3d0)) ; exppow0
	    :exponential-power :a 3.7d0 :b 0.3d0))
  (lisp-unit::assert-true
   (testpdf (lambda (r) (exponential-power-pdf r 3.7d0 1.0d0)) ; exppow1
	    :exponential-power :a 3.7d0 :b 1.0d0))
  (lisp-unit::assert-true
   (testpdf (lambda (r) (exponential-power-pdf r 3.7d0 1.9d0)) ; exppow1a
	    :exponential-power :a 3.7d0 :b 1.9d0))
  (lisp-unit::assert-true
   (testpdf (lambda (r) (exponential-power-pdf r 3.7d0 2.0d0)) ; exppow2
	    :exponential-power :a 3.7d0 :b 2.0d0))
  (lisp-unit::assert-true
   (testpdf (lambda (r) (exponential-power-pdf r 3.7d0 3.5d0)) ; exppow2a
	    :exponential-power :a 3.7d0 :b 3.5d0))
  (lisp-unit::assert-true
   (testpdf (lambda (r) (exponential-power-pdf r 3.7d0 7.5d0)) ; exppow2b
	    :exponential-power :a 3.7d0 :b 7.5d0))
  ;; Automatically converted from cdf/test.c
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-P -1000.0d0 0.7d0 1.8d0) 0.0d0 +TEST-TOL6+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-P -0.1d0 0.7d0 1.8d0) 0.42053490828675155d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-P -1.d-32 0.7d0 1.8d0) 0.5d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-P 0.1d0 0.7d0 1.8d0) 0.5794650917132484d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-P 1.d-32 0.7d0 1.8d0) 0.5d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-P 1000.0d0 0.7d0 1.8d0) 1.0d0 +TEST-TOL6+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-Q -1000.0d0 0.7d0 1.8d0) 1.0d0 +TEST-TOL6+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-Q -0.1d0 0.7d0 1.8d0) 0.5794650917132484d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-Q -1.d-32 0.7d0 1.8d0) 0.5d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-Q 0.1d0 0.7d0 1.8d0) 0.42053490828675155d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-Q 1.d-32 0.7d0 1.8d0) 0.5d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-POWER-Q 1000.0d0 0.7d0 1.8d0) 0.0d0 +TEST-TOL6+))

