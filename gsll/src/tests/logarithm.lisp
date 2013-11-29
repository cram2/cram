;; Regression test LOGARITHM for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST LOGARITHM
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.6931471805599453d0 3.078191837246648d-16)
                        (MULTIPLE-VALUE-LIST (GSL-LOG 2.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #C(0.34657359027997264d0 0.7853981633974483d0)
                              #C(1.539095918623324d-16 7.69547959311662d-17))
                        (MULTIPLE-VALUE-LIST (GSL-LOG #C(1.0d0 1.0d0))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.6931471805599453d0 3.078191837246648d-16)
                        (MULTIPLE-VALUE-LIST (LOG-ABS -2.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 9.999500033330834d-5 2.2203350343487824d-20)
                        (MULTIPLE-VALUE-LIST (LOG-1+X 1.d-4)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -4.999666691664667d-9 1.1101490153075193d-24)
                        (MULTIPLE-VALUE-LIST (LOG-1+X-M1 1.d-4))))

