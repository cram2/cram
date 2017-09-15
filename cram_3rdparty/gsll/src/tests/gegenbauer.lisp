;; Regression test GEGENBAUER for GSLL, automatically generated
;;
;; Copyright 2009, 2014 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST GEGENBAUER
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 6.0d0 5.329070518200751d-15)
                        (MULTIPLE-VALUE-LIST (GEGENBAUER-1 1.0d0 3.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 35.0d0 1.5765166949677223d-14)
                        (MULTIPLE-VALUE-LIST (GEGENBAUER-2 1.0d0 3.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 204.0d0 9.126033262418787d-14)
                        (MULTIPLE-VALUE-LIST (GEGENBAUER-3 1.0d0 3.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1189.0d0 1.056044141023449d-12)
                        (MULTIPLE-VALUE-LIST (GEGENBAUER 4 1.0d0 3.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(1.0d0 6.0d0 35.0d0 204.0d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((ARR (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS 4)))
                           (GEGENBAUER-ARRAY 1.0d0 3.0d0 ARR)
                           (GRID:COPY-TO ARR 'array 'double-float)))))

