;; Test math functions on numbers
;; Liam Healy 2010-12-24 13:54:53EST numbers.lisp
;; Time-stamp: <2014-01-05 12:15:20EST numbers.lisp>

;; Copyright 2011, 2013, 2014 Liam M. Healy
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

(in-package :antik)

(lisp-unit:define-test numbers
    (lisp-unit::assert-numerical-equal
     2.0d0
     (+ 1.0d0 1.0d0))
  (lisp-unit::assert-numerical-equal ; addition 1 argument
   1.0d0
   (+ 1.0d0))
  (lisp-unit::assert-numerical-equal ; addition 2 arguments
   0.0d0
   (- 1.0d0 1.0d0))
  (lisp-unit::assert-numerical-equal ; addition 3 arguments
   -1.0d0
   (+ 1.0d0 3.0d0 -5.0d0))
  (lisp-unit::assert-numerical-equal ; subtraction 1 argument
   -1.0d0
   (- 1.0d0))
  (lisp-unit::assert-numerical-equal ; subtraction 2 argument
   2.0d0
   (- 4.0d0 2.0d0))
  (lisp-unit::assert-numerical-equal ; subtraction 3 arguments
   3.0d0
   (- 1.0d0 3.0d0 -5.0d0))
  (lisp-unit::assert-numerical-equal ; multiplication 1 argument
   2.0d0
   (* 2.0d0))
  (lisp-unit::assert-numerical-equal ; multiplication 2 arguments
   -15.0d0
   (* 3.0d0 -5.0d0))
  (lisp-unit::assert-numerical-equal ; multiplication 3 arguments
   -30.0d0
   (* 2.0d0 3.0d0 -5.0d0))
  (lisp-unit::assert-numerical-equal ; division 1 argument
   0.5d0
   (/ 2.0d0))
  (lisp-unit::assert-numerical-equal ; division 2 arguments
   0.5d0
   (/ 1.0d0 2.0d0))
  (lisp-unit::assert-numerical-equal ; division 3 arguments
   0.1d0
   (/ 2.0d0 4.0d0 5.0d0))
  (lisp-unit::assert-numerical-equal
   2.0d0
   (sqrt 4.0d0))
  (lisp-unit::assert-numerical-equal
   1.0d0
   (cos 0.0d0))
  (lisp-unit::assert-numerical-equal
   0.0d0
   (sin 0.0d0))
  (lisp-unit::assert-numerical-equal
   0.0d0
   (tan 0.0d0))
  (lisp-unit::assert-numerical-equal
   0.0d0
   ;; (pqval (asin 0.0d0))
   (asin 0.0d0))
  (lisp-unit::assert-numerical-equal
   0.0d0
   ;; (pqval (acos 1.0d0))
   (acos 1.0d0))
  (lisp-unit::assert-numerical-equal
   0.0d0
   ;; (pqval (atan 0.0d0))
   (atan 0.0d0))
  (lisp-unit::assert-numerical-equal
   27.0d0
   (expt 3.0d0 3))
  (lisp-unit:assert-error
   'making-complex-number
   (expt -3.0d0 1/2))
  (lisp-unit::assert-numerical-equal
   2.718281828459045d0
   (exp 1.0d0))
  (lisp-unit::assert-numerical-equal
   1.0d0
   (log 2.718281828459045d0))
  (lisp-unit::assert-numerical-equal
   1.0d0
   (abs -1.0d0))
  (lisp-unit:assert-true
   (> 5.0d0 4.0d0))
  (lisp-unit:assert-false
   (< 5.0d0 4.0d0))
  (lisp-unit:assert-true
   (>= 5.0d0 4.0d0))
  (lisp-unit:assert-false
   (<= 5.0d0 4.0d0))
  (lisp-unit:assert-false
   (= 5.0d0 4.0d0))
  (lisp-unit:assert-true
   (= 4.0d0 4.0d0))
  (lisp-unit:assert-true
   (plusp 4.0d0))
  (lisp-unit:assert-true
   (minusp -4.0d0))
  (lisp-unit:assert-false
   (zerop -4.0d0))
  (lisp-unit:assert-true
   (zerop 0.0d0))
  (lisp-unit::assert-numerical-equal
   2.0d0
   (floor 5.0d0 2.0d0))
  (lisp-unit::assert-numerical-equal
   2.0d0
   (round 5.0d0 3.0d0))
  (lisp-unit::assert-numerical-equal
   -1.0d0
   (signum -5.0d0))
  (lisp-unit::assert-numerical-equal
   -1.0d0
   (min -1.0d0 19.3d0 12.25d0))
  (lisp-unit::assert-numerical-equal
   19.3d0
   (max -1.0d0 19.3d0 12.25d0)))
