;; Regression test ELLIPTIC-INTEGRALS for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST ELLIPTIC-INTEGRALS
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.5707963267948966d0 4.598091522633788d-16)
                        (MULTIPLE-VALUE-LIST
                         (ELLIPTIC-INTEGRAL-K-COMPLETE 0.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.5707963267948966d0 3.487868498008632d-16)
                        (MULTIPLE-VALUE-LIST
                         (ELLIPTIC-INTEGRAL-E-COMPLETE 0.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.6774175382039307d0 3.008338192795582d-16)
                        (MULTIPLE-VALUE-LIST
                         (ELLIPTIC-INTEGRAL-F -0.5d0 2.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.4018194805534952d0 4.232239429377521d-16)
                        (MULTIPLE-VALUE-LIST
                         (ELLIPTIC-INTEGRAL-E -0.5d0 2.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.617791316339182d0 3.273131810338189d-16)
                        (MULTIPLE-VALUE-LIST
                         (ELLIPTIC-INTEGRAL-P -0.5d0 2.0d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.06889951441260889d0 3.059753091454848d-17)
                        (MULTIPLE-VALUE-LIST
                         (ELLIPTIC-INTEGRAL-D -0.5d0 2.0d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.8813735870195432d0 1.9570424992111216d-16)
                        (MULTIPLE-VALUE-LIST
                         (ELLIPTIC-INTEGRAL-RC 2.0d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.7992599630303281d0 1.7747136272346433d-16)
                        (MULTIPLE-VALUE-LIST
                         (ELLIPTIC-INTEGRAL-RD 2.0d0 1.0d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.8813735870195432d0 1.9570424992111216d-16)
                        (MULTIPLE-VALUE-LIST
                         (ELLIPTIC-INTEGRAL-RF 2.0d0 1.0d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.5228004174989865d0 1.160850121582039d-16)
                        (MULTIPLE-VALUE-LIST
                         (ELLIPTIC-INTEGRAL-RJ 2.0d0 1.0d0 1.0d0 2.0d0))))

