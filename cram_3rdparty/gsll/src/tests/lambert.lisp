;; Regression test LAMBERT for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST LAMBERT
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.5671432904097838d0 2.518622157098455d-15)
                        (MULTIPLE-VALUE-LIST (LAMBERT-W0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.5671432904097838d0 2.518622157098455d-15)
                        (MULTIPLE-VALUE-LIST (LAMBERT-WM1 1.0d0))))

