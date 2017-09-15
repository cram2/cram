;; Regression test RANK-1-UPDATE for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST RANK-1-UPDATE
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((33772.18f0 17310.045f0 -44960.434f0)
	(-13648.694f0 -6946.5283f0 18135.074f0)
	(34314.832f0 17523.164f0 -45541.547f0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT
	    :INITIAL-CONTENTS
	    '((-34.5f0 8.24f0 3.29f0)
	      (-8.93f0 34.12f0 -6.15f0)
	      (49.27f0 -13.49f0 32.5f0))))
	  (V1
	   (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS '(42.73f0 -17.24f0 43.31f0)))
	  (V2
	   (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS '(-16.12f0 -8.25f0 21.44f0)))
	  (S1 -49.08f0))
      (GRID:COPY-TO (RANK-1-UPDATE S1 V1 V2 M1) 'array 'single-float))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((33772.17700799999d0 17310.044299999998d0 -44960.429295999995d0)
	(-13648.693903999998d0 -6946.528399999999d0 18135.074447999996d0)
	(34314.825376d0 17523.1621d0 -45541.53891200001d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT
	    :INITIAL-CONTENTS
	    '((-34.5d0 8.24d0 3.29d0)
	      (-8.93d0 34.12d0 -6.15d0)
	      (49.27d0 -13.49d0 32.5d0))))
	  (V1
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS '(42.73d0 -17.24d0 43.31d0)))
	  (V2
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS '(-16.12d0 -8.25d0 21.44d0)))
	  (S1 -49.08d0))
      (GRID:COPY-TO (RANK-1-UPDATE S1 V1 V2 M1) 'array 'double-float))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(37793.65f0 36629.23f0) #C(-101265.76f0 118286.42f0)
	   #C(68357.26f0 171191.28f0))
	(#C(36918.79f0 37780.418f0) #C(-104760.06f0 115889.945f0)
	   #C(63820.45f0 173533.31f0))
	(#C(-26166.797f0 1319.2356f0) #C(-8642.514f0 -77109.22f0)
	   #C(-85661.76f0 -33283.21f0))))
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
	  (S1 #C(-49.08f0 -39.66f0)))
      (GRID:COPY-TO (RANK-1-UPDATE S1 V1 V2 M1) 'array '(complex single-float)))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(37793.646065999994d0 36629.23161199999d0)
	   #C(-101265.76059999998d0 118286.41839999997d0)
	   #C(68357.25449199998d0 171191.29244399996d0))
	(#C(36918.78463d0 37780.41610000001d0)
	   #C(-104760.05796d0 115889.92672d0) #C(63820.44154d0 173533.30234d0))
	(#C(-26166.794498000003d0 1319.234524000003d0)
	   #C(-8642.510840000003d0 -77109.20672d0)
	   #C(-85661.74775600001d0 -33283.210252d0))))
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
	    '(#C(42.73d0 -17.24d0)
	      #C(43.31d0 -16.12d0)
	      #C(-8.25d0 21.44d0))))
	  (V2
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-16.12d0 -8.25d0)
	      #C(21.44d0 -49.08d0)
	      #C(-39.66d0 -49.46d0))))
	  (S1 #C(-49.08d0 -39.66d0)))
      (GRID:COPY-TO (RANK-1-UPDATE S1 V1 V2 M1) 'array '(complex double-float))))))

