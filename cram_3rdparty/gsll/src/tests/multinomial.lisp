;; Regression test MULTINOMIAL for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MULTINOMIAL
    (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
     (LIST #(5 0 1 2))
     (MULTIPLE-VALUE-LIST
	 (let ((rng (make-random-number-generator +mt19937+ 0))
	       (p
		(grid:make-foreign-array
		 'double-float :initial-contents '(0.1d0 0.2d0 0.3d0 0.4d0))))
	   (grid:copy-to (sample rng :multinomial :sum 8 :probabilities p) 'array 'fixnum))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 8.064000000000026d-5)
   (MULTIPLE-VALUE-LIST
       (LET ((P
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(0.1d0 0.2d0 0.3d0 0.4d0)))
	     (N
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 32)
				       :INITIAL-CONTENTS '(5 0 1 2))))
	 (MULTINOMIAL-PDF P N))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST -9.425515753641212d0)
   (MULTIPLE-VALUE-LIST
       (LET ((P
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(0.1d0 0.2d0 0.3d0 0.4d0)))
	     (N
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 32)
				       :INITIAL-CONTENTS '(5 0 1 2))))
	 (MULTINOMIAL-LOG-PDF P N)))))

