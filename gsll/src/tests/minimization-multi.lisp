;; Regression test MINIMIZATION-MULTI for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MINIMIZATION-MULTI
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST 0.9920430849306285d0 1.9969168063253164d0
                              30.000823246638923d0)
                        (MULTIPLE-VALUE-LIST
                         (MULTIMIN-EXAMPLE-NO-DERIVATIVE
                          +SIMPLEX-NELDER-MEAD-ON2+ NIL)))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST 0.9999999999999997d0 2.0d0 30.0d0)
                        (MULTIPLE-VALUE-LIST
                         (MULTIMIN-EXAMPLE-DERIVATIVE
                          +CONJUGATE-FLETCHER-REEVES+ NIL)))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST 0.9999999999999997d0 2.0d0 30.0d0)
                        (MULTIPLE-VALUE-LIST
                         (MULTIMIN-EXAMPLE-DERIVATIVE +CONJUGATE-POLAK-RIBIERE+
                                                      NIL)))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST 0.9999999999999997d0 2.0d0 30.0d0)
                        (MULTIPLE-VALUE-LIST
                         (MULTIMIN-EXAMPLE-DERIVATIVE +VECTOR-BFGS+ NIL)))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST 0.9999999999998208d0 1.9999999999995521d0 30.0d0)
                        (MULTIPLE-VALUE-LIST
                         (MULTIMIN-EXAMPLE-DERIVATIVE +VECTOR-BFGS2+ NIL)))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST 0.9999999999998208d0 1.9999999999995521d0 30.0d0)
                        (MULTIPLE-VALUE-LIST
                         (MULTIMIN-EXAMPLE-DERIVATIVE-SCALARS +VECTOR-BFGS2+
                                                              NIL))))

