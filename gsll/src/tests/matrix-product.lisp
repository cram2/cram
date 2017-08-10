;; Regression test MATRIX-PRODUCT for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MATRIX-PRODUCT
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((74519.41f0 -16747.695f0 61311.69f0)
	(27307.273f0 -6133.5903f0 -25711.75f0)
	(-29088.719f0 83072.016f0 -11019.719f0)))
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
	     (M3
	      (GRID:MAKE-FOREIGN-ARRAY
	       'SINGLE-FLOAT :INITIAL-CONTENTS
	       '((19.68f0 -5.55f0 -8.82f0)
		 (25.37f0 -30.58f0 31.67f0)
		 (29.36f0 -33.24f0 -27.03f0))))
	     (S1 -41.67f0)
	     (S2 42.0f0))
	 (GRID:COPY-TO (MATRIX-PRODUCT M1 M2 M3 S1 S2) 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((74519.41329d0 -16747.696061999995d0 61311.694176d0)
	(27307.276670999996d0 -6133.589574d0 -25711.752344999997d0)
	(-29088.718053000033d0 83072.022741d0 -11019.721527000016d0)))
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
	     (M3
	      (GRID:MAKE-FOREIGN-ARRAY
	       'DOUBLE-FLOAT :INITIAL-CONTENTS
	       '((19.68d0 -5.55d0 -8.82d0)
		 (25.37d0 -30.58d0 31.67d0)
		 (29.36d0 -33.24d0 -27.03d0))))
	     (S1 -41.67d0)
	     (S2 42.0d0))
	 (GRID:COPY-TO (MATRIX-PRODUCT M1 M2 M3 S1 S2) 'array 'double-float))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(140927.64f0 -143005.16f0) #C(72986.305f0 -201162.86f0)
	   #C(65956.16f0 9034.215f0))
	(#C(-102500.125f0 118033.82f0) #C(-147775.0f0 58489.473f0)
	   #C(-9582.236f0 190719.98f0))
	(#C(-16551.412f0 336505.22f0) #C(-7668.242f0 385798.25f0)
	   #C(45313.527f0 140641.6f0))))
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
	     (M3
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX SINGLE-FLOAT)
	       :INITIAL-CONTENTS
	       '((#C(19.68f0 -5.55f0) #C(-8.82f0 25.37f0) #C(-30.58f0 31.67f0))
		 (#C(25.37f0 -30.58f0) #C(31.67f0 29.36f0) #C(-33.24f0 -27.03f0))
		 (#C(29.36f0 -33.24f0) #C(-27.03f0 -41.67f0) #C(42.0f0 -20.81f0)))))
	     (S1 #C(-41.67f0 42.0f0))
	     (S2 #C(42.0f0 -20.81f0)))
	 (GRID:COPY-TO (MATRIX-PRODUCT M1 M2 M3 S1 S2) 'array '(complex single-float)))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(140927.638311d0 -143005.14965699997d0)
	   #C(72986.314354d0 -201162.870342d0)
	   #C(65956.156396d0 9034.212785d0))
	(#C(-102500.143082d0 118033.81826900001d0)
	   #C(-147775.01365700003d0 58489.469011999994d0)
	   #C(-9582.23530800004d0 190719.98257199998d0))
	(#C(-16551.43698600002d0 336505.21680500003d0)
	   #C(-7668.248943000013d0 385798.22976300004d0)
	   #C(45313.51313299998d0 140641.59851399998d0))))
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
	     (M3
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX DOUBLE-FLOAT)
	       :INITIAL-CONTENTS
	       '((#C(19.68d0 -5.55d0) #C(-8.82d0 25.37d0) #C(-30.58d0 31.67d0))
		 (#C(25.37d0 -30.58d0) #C(31.67d0 29.36d0) #C(-33.24d0 -27.03d0))
		 (#C(29.36d0 -33.24d0) #C(-27.03d0 -41.67d0) #C(42.0d0 -20.81d0)))))
	     (S1 #C(-41.67d0 42.0d0))
	     (S2 #C(42.0d0 -20.81d0)))
	 (GRID:COPY-TO (MATRIX-PRODUCT M1 M2 M3 S1 S2) 'array '(complex double-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(72971.1f0 60998.137f0 -184676.98f0))
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
	     (V2
	      (GRID:MAKE-FOREIGN-ARRAY
	       'SINGLE-FLOAT :INITIAL-CONTENTS
	       '(-16.12f0 -8.25f0 21.44f0)))
	     (S1 -49.08f0)
	     (S2 -39.66f0))
	 (GRID:COPY-TO (MATRIX-PRODUCT M1 V1 V2 S1 S2) 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(72971.10171599999d0 60998.13393599999d0
      -184676.981676d0))
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
	     (V2
	      (GRID:MAKE-FOREIGN-ARRAY
	       'DOUBLE-FLOAT :INITIAL-CONTENTS
	       '(-16.12d0 -8.25d0 21.44d0)))
	     (S1 -49.08d0)
	     (S2 -39.66d0))
	 (GRID:COPY-TO (MATRIX-PRODUCT M1 V1 V2 S1 S2) 'array 'double-float))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(124163.57f0 -3332.2988f0) #C(119793.48f0 -166378.0f0)
      #C(-189845.23f0 -81764.12f0)))
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
	     (V2
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX SINGLE-FLOAT)
	       :INITIAL-CONTENTS
	       '(#C(-16.12f0 -8.25f0) #C(21.44f0 -49.08f0) #C(-39.66f0 -49.46f0))))
	     (S1 #C(-49.08f0 -39.66f0))
	     (S2 #C(-39.66f0 -49.46f0)))
	 (GRID:COPY-TO (MATRIX-PRODUCT M1 V1 V2 S1 S2) 'array '(complex single-float)))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(124163.58013199999d0 -3332.2974459999896d0)
      #C(119793.47618999999d0 -166378.00423d0)
      #C(-189845.21679399998d0 -81764.10481799999d0)))
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
	     (V2
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX DOUBLE-FLOAT)
	       :INITIAL-CONTENTS
	       '(#C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0) #C(-39.66d0 -49.46d0))))
	     (S1 #C(-49.08d0 -39.66d0))
	     (S2 #C(-39.66d0 -49.46d0)))
	 (GRID:COPY-TO (MATRIX-PRODUCT M1 V1 V2 S1 S2) 'array '(complex double-float))))))
