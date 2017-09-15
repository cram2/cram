;; Regression test MATHEMATICAL for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MATHEMATICAL
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 9.995003330835331d-4)
                        (MULTIPLE-VALUE-LIST (LOG+1 0.001d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.0010005001667083417d0)
                        (MULTIPLE-VALUE-LIST (EXP-1 0.001d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 5.0d0 2.220446049250313d-15)
                        (MULTIPLE-VALUE-LIST (HYPOTENUSE 3.0d0 4.0d0))))

