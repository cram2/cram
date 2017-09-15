;; Regression test LAGUERRE for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST LAGUERRE
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -1.0d0 2.220446049250313d-15)
                        (MULTIPLE-VALUE-LIST (LAGUERRE-1 1.0d0 3.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -1.5d0 1.7985612998927536d-14)
                        (MULTIPLE-VALUE-LIST (LAGUERRE-2 1.0d0 3.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.5d0 6.59472476627343d-14)
                        (MULTIPLE-VALUE-LIST (LAGUERRE-3 1.0d0 3.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.875d0 6.793349802129357d-14)
                        (MULTIPLE-VALUE-LIST (LAGUERRE 4 1.0d0 3.0d0))))

