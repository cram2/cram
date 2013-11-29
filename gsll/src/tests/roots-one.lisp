;; Regression test ROOTS-ONE for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST ROOTS-ONE
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 2.2357177734375d0)
   (MULTIPLE-VALUE-LIST
    (ROOTS-ONE-EXAMPLE-NO-DERIVATIVE +BISECTION-FSOLVER+ NIL)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 2.23606797749979d0)
   (MULTIPLE-VALUE-LIST
    (ROOTS-ONE-EXAMPLE-NO-DERIVATIVE +FALSE-POSITION-FSOLVER+ NIL)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 2.2360634081902244d0)
   (MULTIPLE-VALUE-LIST
    (ROOTS-ONE-EXAMPLE-NO-DERIVATIVE +BRENT-FSOLVER+ NIL)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 2.236067977499978d0)
   (MULTIPLE-VALUE-LIST
    (ROOTS-ONE-EXAMPLE-DERIVATIVE +NEWTON-FDFSOLVER+ NIL)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 2.2360679849648637d0)
   (MULTIPLE-VALUE-LIST
    (ROOTS-ONE-EXAMPLE-DERIVATIVE +SECANT-FDFSOLVER+ NIL)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 2.23606797749979d0)
   (MULTIPLE-VALUE-LIST
    (ROOTS-ONE-EXAMPLE-DERIVATIVE +STEFFENSON-FDFSOLVER+ NIL))))
