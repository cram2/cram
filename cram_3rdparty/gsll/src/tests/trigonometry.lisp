;; Regression test TRIGONOMETRY for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST TRIGONOMETRY
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.8414709848078965d0 3.736881847550928d-16)
                        (MULTIPLE-VALUE-LIST (GSL-SIN 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #C(1.2984575814159773d0 0.6349639147847361d0)
                              #C(5.766310013548447d-16 2.8198062320005596d-16))
                        (MULTIPLE-VALUE-LIST (GSL-SIN #C(1.0d0 1.0d0))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.5403023058681398d0 2.3994242409314904d-16)
                        (MULTIPLE-VALUE-LIST (GSL-COS 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #C(0.8337300251311491d0 -0.9888977057628651d0)
                              #C(3.7025050808876487d-16 4.3915880077477046d-16))
                        (MULTIPLE-VALUE-LIST (GSL-COS #C(1.0d0 1.0d0))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 2.23606797749979d0 9.930136612989092d-16)
                        (MULTIPLE-VALUE-LIST (HYPOTENUSE 1.0d0 2.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.6366197723675814d0 3.0072729231305663d-16)
                        (MULTIPLE-VALUE-LIST (SINC 0.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #C(0.3683837314249251d0 0.4548202333099499d0)
                              #C(1.6359524021011266d-16 8.179762010505633d-17))
                        (MULTIPLE-VALUE-LIST (LOG-SIN #C(1.0d0 1.0d0))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.6518223259470272d0 2.8946726169244526d-16)
                        (MULTIPLE-VALUE-LIST (LOG-SINH 0.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.12011450695827751d0 4.401938023864669d-17)
                        (MULTIPLE-VALUE-LIST (LOG-COSH 0.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.0806046117362795d0 1.682941969615793d0
                              8.535730329413909d-16 9.873187936033346d-16)
                        (MULTIPLE-VALUE-LIST
                         (POLAR-TO-RECTANGULAR 2.0d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 2.23606797749979d0 0.4636476090008061d0
                              9.930136612989092d-16 2.0590090033003876d-16)
                        (MULTIPLE-VALUE-LIST
                         (RECTANGULAR-TO-POLAR 2.0d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -1.2831853071795865d0)
                        (MULTIPLE-VALUE-LIST (RESTRICT-SYMMETRIC 5.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 5.283185307179586d0)
                        (MULTIPLE-VALUE-LIST (RESTRICT-POSITIVE -1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.479425538604203d0 0.008775825618904047d0)
                        (MULTIPLE-VALUE-LIST (SIN-ERR 0.5d0 0.01d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.8775825618903728d0 0.004794255386042614d0)
                        (MULTIPLE-VALUE-LIST (COS-ERR 0.5d0 0.01d0))))

