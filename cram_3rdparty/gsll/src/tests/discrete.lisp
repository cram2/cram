;; Regression test DISCRETE for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST DISCRETE
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST (LIST 1 0 1 1 0 1 1 2 1 2 2))
   (MULTIPLE-VALUE-LIST
    (LET* ((PROBABILITIES
	    (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
			 '(0.25d0 0.5d0 0.25d0)))
	   (TABLE (MAKE-DISCRETE-RANDOM PROBABILITIES))
	   (RNG
	    (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 10 COLLECT
	   (sample rng :discrete :table table)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.5d0)
   (MULTIPLE-VALUE-LIST
    (LET* ((PROBABILITIES
	    (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS '(0.25d0 0.5d0 0.25d0)))
	   (TABLE (MAKE-DISCRETE-RANDOM PROBABILITIES)))
      (DISCRETE-PDF 1 TABLE)))))

