;; Regression test LEVY for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST LEVY
  ;; From randist/test.c
  (lisp-unit::assert-true
   (testpdf (lambda (r) (cauchy-pdf r 5.0d0)) :levy :c 5.0d0 :alpha 1.0d0)) ; levy1
  (lisp-unit::assert-true
   (testpdf (lambda (r) (cauchy-pdf r 5.0d0)) :levy :c 5.0d0 :alpha 1.01d0)) ; levy1a
  (lisp-unit::assert-true
   (testpdf (lambda (r) (gaussian-pdf r (* (sqrt 2.0d0) 5.0d0)))
	    :levy :c 5.0d0 :alpha 2.0d0)) ; levy2
  (lisp-unit::assert-true
   (testpdf (lambda (r) (gaussian-pdf r (* (sqrt 2.0d0) 5.0d0)))
	    :levy :c 5.0d0 :alpha 1.99d0)) ; levy2a
  (lisp-unit::assert-true
   (testpdf (lambda (r) (cauchy-pdf r 5.0d0))
	    :levy-skew :c 5.0d0 :alpha 1.0d0 :beta 0.0d0)) ; levy_skew1
  (lisp-unit::assert-true
   (testpdf (lambda (r) (cauchy-pdf r 5.0d0))
	    :levy-skew :c 5.0d0 :alpha 1.01d0 :beta 0.0d0)) ; levy_skew1a
  (lisp-unit::assert-true
   (testpdf (lambda (r) (gaussian-pdf r (* (sqrt 2.0d0) 5.0d0)))
	    :levy-skew :c 5.0d0 :alpha 2.0d0 :beta 0.0d0)) ; levy_skew2
  (lisp-unit::assert-true
   (testpdf (lambda (r) (gaussian-pdf r (* (sqrt 2.0d0) 5.0d0)))
	    :levy-skew :c 5.0d0 :alpha 1.99d0 :beta 0.0d0)) ; levy_skew2a
  (lisp-unit::assert-true
   (testpdf (lambda (r) (cauchy-pdf r 5.0d0))
	    :levy-skew :c 5.0d0 :alpha 1.01d0 :beta 0.001d0)) ; levy_skew1b
  (lisp-unit::assert-true
   (testpdf (lambda (r) (gaussian-pdf r (* (sqrt 2.0d0) 5.0d0)))
	    :levy-skew :c 5.0d0 :alpha 1.99d0 :beta 0.001d0))) ; levy_skew2b

