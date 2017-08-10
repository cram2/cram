;; Regression test MATRIX-PRODUCT-NONSQUARE for GSLL, automatically generated
;;
;; Copyright 2009, 2010, 2011, 2014 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST MATRIX-PRODUCT-NONSQUARE
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-1488.7347f0 959.9901f0) (774.94495f0 1312.0566f0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS
	    '((-34.5f0 8.24f0 3.29f0)
	      (-8.93f0 34.12f0 -6.15f0))))
	  (M2
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS
	    '((49.27f0 -13.49f0) (32.5f0 42.73f0)
	      (-17.24f0 43.31f0)))))
      (GRID:COPY-TO (MATRIX-PRODUCT M1 M2) 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-1488.7346d0 959.9901d0)
	(774.9448999999998d0 1312.0567999999996d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS
	    '((-34.5d0 8.24d0 3.29d0)
	      (-8.93d0 34.12d0 -6.15d0))))
	  (M2
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS
	    '((49.27d0 -13.49d0)
	      (32.5d0 42.73d0)
	      (-17.24d0 43.31d0)))))
      (GRID:COPY-TO (MATRIX-PRODUCT M1 M2) 'array 'double-float))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(-1422.0259f0 2305.5098f0)
	   #C(-1744.0583f0 -1092.294f0))
	(#C(-3459.892f0 1995.4918f0)
	   #C(-3290.4465f0 -801.0577f0))))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(34.12f0 -6.15f0))
	      (#C(-8.93f0 34.12f0) #C(-6.15f0 49.27f0) #C(-13.49f0 32.5f0)))))
	  (M2
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(49.27f0 -13.49f0) #C(32.5f0 42.73f0))
	      (#C(32.5f0 42.73f0) #C(-17.24f0 43.31f0))
	      (#C(-17.24f0 43.31f0) #C(-16.12f0 -8.25f0))))))
      (GRID:COPY-TO (MATRIX-PRODUCT M1 M2) 'array '(complex single-float)))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(-1422.0258d0 2305.5097000000005d0)
	   #C(-1744.0584d0 -1092.2939d0))
	(#C(-3459.8918d0 1995.4917d0)
	   #C(-3290.4465d0 -801.0577000000002d0))))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0))
	      (#C(-8.93d0 34.12d0) #C(-6.15d0 49.27d0) #C(-13.49d0 32.5d0)))))
	  (M2
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(49.27d0 -13.49d0) #C(32.5d0 42.73d0))
	      (#C(32.5d0 42.73d0) #C(-17.24d0 43.31d0))
	      (#C(-17.24d0 43.31d0) #C(-16.12d0 -8.25d0))))))
      (GRID:COPY-TO (MATRIX-PRODUCT M1 M2) 'array '(complex double-float))))))

