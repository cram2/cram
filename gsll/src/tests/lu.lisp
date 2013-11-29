;; Regression test LU for GSLL, automatically generated
;; with some manual changes to the results
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

;;; Answers (except for invert-matrix)inserted from linalg/test.c
;;; GSL has #define GSL_DBL_EPSILON        2.2204460492503131e-16
;;; which is 2x what double-float-epsilon is.
(LISP-UNIT:DEFINE-TEST LU
  (let ((lisp-unit:*epsilon* (* 2 8 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb2-soln*)
     (MULTIPLE-VALUE-LIST (TEST-LU-SOLVE-DIM *hilb2*))))
  (let ((lisp-unit:*epsilon* (* 2 64 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb3-soln*)
     (MULTIPLE-VALUE-LIST (TEST-LU-SOLVE-DIM *hilb3*))))
  (let ((lisp-unit:*epsilon* (* 2 2048 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb4-soln*)
     (MULTIPLE-VALUE-LIST (TEST-LU-SOLVE-DIM *hilb4*))))
  (let ((lisp-unit:*epsilon* 0.5d0))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb12-soln*)
     (MULTIPLE-VALUE-LIST (TEST-LU-SOLVE-DIM *hilb12*))))
  (let ((lisp-unit:*epsilon* (* 2 8 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander2-soln*)
     (MULTIPLE-VALUE-LIST (TEST-LU-SOLVE-DIM *vander2*))))
  (let ((lisp-unit:*epsilon* (* 2 64 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander3-soln*)
     (MULTIPLE-VALUE-LIST (TEST-LU-SOLVE-DIM *vander3*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander4-soln*)
     (MULTIPLE-VALUE-LIST (TEST-LU-SOLVE-DIM *vander4*))))
  (let ((lisp-unit:*epsilon* 0.05d0))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander12-soln*)
     (MULTIPLE-VALUE-LIST (TEST-LU-SOLVE-DIM *vander12*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST
      (GRID:MAKE-FOREIGN-ARRAY
       '(COMPLEX DOUBLE-FLOAT) :INITIAL-CONTENTS
       '(#C(2.40717272023734d+01 -9.84612797621247d+00)
	 #C(-2.69338853034031d+02 8.75455232472528d+01)
	 #C(2.96661356736296d+03 -1.02624473923993d+03)
	 #C(-1.82073812124749d+04 5.67384473042410d+03)
	 #C(5.57693879019068d+04 -1.61540963210502d+04)
	 #C(-7.88941207561151d+04 1.95053812987858d+04)
	 #C(3.95548551241728d+04 -7.76593696255317d+03))))
     (MULTIPLE-VALUE-LIST
      (TEST-LU-SOLVE-DIM (CREATE-COMPLEX-MATRIX 7)))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST
    #(-39.65999999999999d0 -49.46000000000001d0
      19.679999999999993d0 -5.549999999999997d0))
   (MULTIPLE-VALUE-LIST
    (LET ((MATRIX
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS
	    '((-34.5d0 8.24d0 3.29d0 -8.93d0)
	      (34.12d0 -6.15d0 49.27d0 -13.49d0)
	      (32.5d0 42.73d0 -17.24d0 43.31d0)
	      (-16.12d0 -8.25d0 21.44d0 -49.08d0))))
	  (VEC
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				    '(-39.66d0 -49.46d0 19.68d0 -5.55d0))))
      (MULTIPLE-VALUE-BIND
	    (MATRIX PERM)
	  (LU-DECOMPOSITION MATRIX)
	(LET ((X (LU-SOLVE MATRIX VEC PERM)))
	  (GRID:COPY-TO
	   (PERMUTE-INVERSE PERM
			    (MATRIX-PRODUCT-TRIANGULAR
			     MATRIX
			     (MATRIX-PRODUCT-TRIANGULAR
			      MATRIX X 1 :UPPER :NOTRANS
			      :NONUNIT)
			     1 :LOWER :NOTRANS
			     :UNIT))))))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-39.65999999999999d0 -49.46000000000001d0)
      #C(19.679999999999996d0 -5.549999999999995d0)
      #C(-8.820000000000006d0 25.370000000000005d0)
      #C(-30.580000000000002d0 31.67d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((MATRIX
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0) #C(49.27d0 -13.49d0))
	      (#C(34.12d0 -6.15d0) #C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0))
	      (#C(32.5d0 42.73d0) #C(-17.24d0 43.31d0) #C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0))
	      (#C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0) #C(-39.66d0 -49.46d0) #C(19.68d0 -5.55d0)))))
	  (VEC
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-39.66d0 -49.46d0) #C(19.68d0 -5.55d0)
	      #C(-8.82d0 25.37d0) #C(-30.58d0 31.67d0)))))
      (MULTIPLE-VALUE-BIND
	    (MATRIX PERM)
	  (LU-DECOMPOSITION MATRIX)
	(LET ((X (LU-SOLVE MATRIX VEC PERM)))
	  (GRID:COPY-TO
	   (PERMUTE-INVERSE
	    PERM
	    (MATRIX-PRODUCT-TRIANGULAR
	     MATRIX
	     (MATRIX-PRODUCT-TRIANGULAR MATRIX X 1 :UPPER :NOTRANS :NONUNIT)
	     1 :LOWER :NOTRANS :UNIT))))))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-1.9999999999999998d0 1.0d0)
	(1.4999999999999998d0 -0.49999999999999994d0)))
   (MULTIPLE-VALUE-LIST
    (GRID:COPY-TO
     (INVERT-MATRIX
      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS '(2 2)
			       :INITIAL-CONTENTS
			       '((1.0d0 2.0d0) (3.0d0 4.0d0))))))))
