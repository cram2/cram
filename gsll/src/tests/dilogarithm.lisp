;; Regression test DILOGARITHM for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST DILOGARITHM
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.6449340668482264d0 7.304974700020789d-16)
                        (MULTIPLE-VALUE-LIST (DILOGARITHM 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #C(-0.20561675835602822d0 0.915965594177219d0)
                              #C(2.100180226255977d-15 7.618282373747058d-16))
                        (MULTIPLE-VALUE-LIST (DILOGARITHM #C(0.0d0 1.0d0)))))

