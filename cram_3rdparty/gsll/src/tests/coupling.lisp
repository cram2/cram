;; Regression test COUPLING for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST COUPLING
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.7071067811865475d0 3.14018491736755d-16)
                        (MULTIPLE-VALUE-LIST (COUPLING-3J 0 1 1 0 1 -1)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.408248290463863d0 5.438959822042073d-16)
                        (MULTIPLE-VALUE-LIST (COUPLING-6J 1 1 2 0 2 1)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.1388888888888889d0 6.638400825147663d-16)
                        (MULTIPLE-VALUE-LIST (COUPLING-9J 1 1 2 1 2 1 2 1 1))))

