;; Regression test MATRIX-PRODUCT-TRIANGULAR for GSLL, automatically generated
;;
;; Copyright 2009, 2010, 2014 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST MATRIX-PRODUCT-TRIANGULAR
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-34803.82f0 7799.5503f0 -29131.375f0)
	(-4884.0327f0 -739.594f0 20382.809f0)
	(-31391.57f0 -25366.535f0 -31634.615f0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS
	    '((-34.5f0 8.24f0 3.29f0)
	      (-8.93f0 34.12f0 -6.15f0)
	      (49.27f0 -13.49f0 32.5f0))))
	  (M2
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS
	    '((42.73f0 -17.24f0 43.31f0)
	      (-16.12f0 -8.25f0 21.44f0)
	      (-49.08f0 -39.66f0 -49.46f0))))
	  (S1 19.68f0))
      (GRID:COPY-TO (MATRIX-PRODUCT-TRIANGULAR M1 M2 S1) 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-34803.82416d0 7799.550047999999d0 -29131.375104000002d0)
	(-4884.032832000001d0 -739.5940799999992d0 20382.808224d0)
	(-31391.568d0 -25366.535999999996d0 -31634.616d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS
	    '((-34.5d0 8.24d0 3.29d0)
	      (-8.93d0 34.12d0 -6.15d0)
	      (49.27d0 -13.49d0 32.5d0))))
	  (M2
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS
	    '((42.73d0 -17.24d0 43.31d0)
	      (-16.12d0 -8.25d0 21.44d0)
	      (-49.08d0 -39.66d0 -49.46d0))))
	  (S1 19.68d0))
      (GRID:COPY-TO (MATRIX-PRODUCT-TRIANGULAR M1 M2 S1) 'array 'single-float))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(-66397.9f0 18986.908f0) #C(-56335.145f0 48514.305f0) #C(-18830.469f0 -13449.601f0))
	(#C(38337.09f0 -49128.918f0) #C(42681.344f0 -22972.445f0) #C(50375.406f0 -50562.54f0))
	(#C(42453.223f0 -42606.086f0) #C(-13764.868f0 -48835.813f0) #C(8910.526f0 -4389.117f0))))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(34.12f0 -6.15f0))
	      (#C(-8.93f0 34.12f0) #C(-6.15f0 49.27f0) #C(-13.49f0 32.5f0))
	      (#C(49.27f0 -13.49f0) #C(32.5f0 42.73f0) #C(-17.24f0 43.31f0)))))
	  (M2
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(42.73f0 -17.24f0) #C(43.31f0 -16.12f0) #C(-8.25f0 21.44f0))
	      (#C(-16.12f0 -8.25f0) #C(21.44f0 -49.08f0) #C(-39.66f0 -49.46f0))
	      (#C(-49.08f0 -39.66f0) #C(-49.46f0 19.68f0) #C(-5.55f0 -8.82f0)))))
	  (S1 #C(19.68f0 -5.55f0)))
      (GRID:COPY-TO (MATRIX-PRODUCT-TRIANGULAR M1 M2 S1) 'array '(complex single-float)))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(-66397.89753899998d0 18986.908142999997d0)
	   #C(-56335.14258600001d0 48514.306278000004d0)
	   #C(-18830.468709d0 -13449.603000000003d0))
	(#C(38337.08717099999d0 -49128.917505000005d0)
	   #C(42681.34176d0 -22972.447481999992d0)
	   #C(50375.404416000005d0 -50562.535017d0))
	(#C(42453.21956399999d0 -42606.08134200001d0)
	   #C(-13764.866562000001d0 -48835.809623999994d0)
	   #C(8910.526581d0 -4389.116526d0))))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0))
	      (#C(-8.93d0 34.12d0) #C(-6.15d0 49.27d0) #C(-13.49d0 32.5d0))
	      (#C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)))))
	  (M2
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(42.73d0 -17.24d0) #C(43.31d0 -16.12d0) #C(-8.25d0 21.44d0))
	      (#C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0) #C(-39.66d0 -49.46d0))
	      (#C(-49.08d0 -39.66d0) #C(-49.46d0 19.68d0) #C(-5.55d0 -8.82d0)))))
	  (S1 #C(19.68d0 -5.55d0)))
      (GRID:COPY-TO (MATRIX-PRODUCT-TRIANGULAR M1 M2 S1) 'array '(complex double-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-1473.7527f0 -854.58527f0 1407.5751f0))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS
	    '((-34.5f0 8.24f0 3.29f0)
	      (-8.93f0 34.12f0 -6.15f0)
	      (49.27f0 -13.49f0 32.5f0))))
	  (V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS
	    '(42.73f0 -17.24f0 43.31f0)))
	  (S1 -16.12f0))
      (GRID:COPY-TO (MATRIX-PRODUCT-TRIANGULAR M1 V1 S1) 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-1473.7527d0 -854.5853d0 1407.575d0))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS
	    '((-34.5d0 8.24d0 3.29d0)
	      (-8.93d0 34.12d0 -6.15d0)
	      (49.27d0 -13.49d0 32.5d0))))
	  (V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS
	    '(42.73d0 -17.24d0 43.31d0)))
	  (S1 -16.12d0))
      (GRID:COPY-TO (MATRIX-PRODUCT-TRIANGULAR M1 V1 S1) 'array 'double-float))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-1483.223f0 1289.3523f0) #C(-57.63159f0 1675.6711f0)
      #C(-786.3365f0 -726.9331f0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(34.12f0 -6.15f0))
	      (#C(-8.93f0 34.12f0) #C(-6.15f0 49.27f0) #C(-13.49f0 32.5f0))
	      (#C(49.27f0 -13.49f0) #C(32.5f0 42.73f0) #C(-17.24f0 43.31f0)))))
	  (V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(42.73f0 -17.24f0) #C(43.31f0 -16.12f0) #C(-8.25f0 21.44f0))))
	  (S1 #C(-16.12f0 -8.25f0)))
      (GRID:COPY-TO (MATRIX-PRODUCT-TRIANGULAR M1 V1 S1) 'array '(complex single-float)))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-1483.2230999999997d0 1289.3523999999998d0)
      #C(-57.63160000000005d0 1675.6711000000003d0)
      #C(-786.3364000000001d0 -726.9331d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0))
	      (#C(-8.93d0 34.12d0) #C(-6.15d0 49.27d0) #C(-13.49d0 32.5d0))
	      (#C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)))))
	  (V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(42.73d0 -17.24d0) #C(43.31d0 -16.12d0) #C(-8.25d0 21.44d0))))
	  (S1 #C(-16.12d0 -8.25d0)))
      (GRID:COPY-TO (MATRIX-PRODUCT-TRIANGULAR M1 V1 S1) 'array '(complex double-float))))))

