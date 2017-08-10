;; Regression test QUASI-RANDOM-NUMBER-GENERATORS for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST QUASI-RANDOM-NUMBER-GENERATORS
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST 0.5d0 0.5d0 0.75d0 0.25d0 0.25d0 0.75d0 0.375d0
	  0.375d0 0.875d0 0.875d0))
   (MULTIPLE-VALUE-LIST
    (LET ((GEN (MAKE-QUASI-RANDOM-NUMBER-GENERATOR +SOBOL+ 2))
	  (VEC (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS 2)))
      (LOOP REPEAT 5 DO (QRNG-GET GEN VEC) APPEND
	   (COERCE (GRID:COPY-TO VEC 'array 'double-float) 'LIST))))))
