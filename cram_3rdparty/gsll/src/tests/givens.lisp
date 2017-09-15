;; Regression test GIVENS for GSLL, automatically generated
;;
;; Copyright 2009, 2011, 2014 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST GIVENS
    (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
     (LIST
      (LIST
       #(-47.39726f0 8.24f0 3.29f0 -8.93f0 34.12f0 -6.15f0 49.27f0 -13.49f0)
       #(-0.6856937f0 42.73f0 -17.24f0 43.31f0 -16.12f0 -8.25f0 21.44f0
	 -49.08f0)))
     (MULTIPLE-VALUE-LIST
	 (LET ((V1
		(GRID:MAKE-FOREIGN-ARRAY
		 'SINGLE-FLOAT
		 :INITIAL-CONTENTS
		 '(-34.5f0 8.24f0 3.29f0 -8.93f0 34.12f0 -6.15f0 49.27f0 -13.49f0)))
	       (V2
		(GRID:MAKE-FOREIGN-ARRAY
		 'SINGLE-FLOAT
		 :INITIAL-CONTENTS
		 '(32.5f0 42.73f0 -17.24f0 43.31f0 -16.12f0 -8.25f0 21.44f0 -49.08f0)))
	       (ANGLES
		(GRID:MAKE-FOREIGN-ARRAY
		 'SINGLE-FLOAT
		 :INITIAL-CONTENTS
		 '(-39.66f0 -49.46f0 19.68f0 -5.55f0 -8.82f0 25.37f0 -30.58f0 31.67f0)))
	       (SINES
		(GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :DIMENSIONS '8))
	       (COSINES
		(GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :DIMENSIONS '8)))
	   (LOOP FOR I BELOW 8 DO
	     (SETF (GRID:AREF SINES I) (SIN (GRID:AREF ANGLES I)))
	     (SETF (GRID:AREF COSINES I)
		   (COS (GRID:AREF ANGLES I))))
	   (GIVENS-ROTATION V1 V2 COSINES SINES)
	   (LIST (GRID:COPY-TO V1 'array 'single-float) (GRID:COPY-TO V2 'array 'single-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST
     #(-47.39725730461627d0 8.24d0 3.29d0 -8.93d0 34.12d0
       -6.15d0 49.27d0 -13.49d0)
     #(-0.6856936845760199d0 42.73d0 -17.24d0 43.31d0
       -16.12d0 -8.25d0 21.44d0 -49.08d0)))
   (MULTIPLE-VALUE-LIST
       (LET ((V1
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(-34.5d0 8.24d0 3.29d0 -8.93d0
					 34.12d0 -6.15d0 49.27d0
					 -13.49d0)))
	     (V2
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(32.5d0 42.73d0 -17.24d0 43.31d0
					 -16.12d0 -8.25d0 21.44d0
					 -49.08d0)))
	     (ANGLES
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(-39.66d0 -49.46d0 19.68d0
					 -5.55d0 -8.82d0 25.37d0 -30.58d0
					 31.67d0)))
	     (SINES
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS '8))
	     (COSINES
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS '8)))
	 (LOOP FOR I BELOW 8 DO
	   (SETF (GRID:AREF SINES I) (SIN (GRID:AREF ANGLES I)))
	   (SETF (GRID:AREF COSINES I)
		 (COS (GRID:AREF ANGLES I))))
	 (GIVENS-ROTATION V1 V2 COSINES SINES)
	 (LIST (GRID:COPY-TO V1 'array 'double-float) (GRID:COPY-TO V2 'array 'double-float))))))

