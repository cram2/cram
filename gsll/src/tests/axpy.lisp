;; Regression test AXPY for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST AXPY
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(1400.77 -284.0684 -147.7214 397.47382 -1369.3191
      235.659 -1932.6083 485.93335))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS
	    '(-34.5 8.24 3.29 -8.93 34.12 -6.15 49.27 -13.49)))
	  (V2
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS
	    '(32.5 42.73 -17.24 43.31 -16.12 -8.25 21.44 -49.08)))
	  (SCALAR -39.66))
      (grid:copy-to (AXPY SCALAR V1 V2)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(1400.77d0 -284.06839999999994d0 -147.7214d0
      397.4738d0 -1369.3191999999997d0 235.659d0
      -1932.6082d0 485.93339999999995d0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT
	    :INITIAL-CONTENTS
	    '(-34.5d0 8.24d0 3.29d0 -8.93d0
	      34.12d0 -6.15d0 49.27d0
	      -13.49d0)))
	  (V2
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT
	    :INITIAL-CONTENTS
	    '(32.5d0 42.73d0 -17.24d0 43.31d0
	      -16.12d0 -8.25d0 21.44d0
	      -49.08d0)))
	  (SCALAR -39.66d0))
      (GRID:COPY-TO (AXPY SCALAR V1 V2)))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(1808.3204 1422.3015) #C(-589.3992 234.75043)
      #C(-1673.498 -1451.916) #C(-2599.8237 -1950.9608)
      #C(784.81586 -3351.5815) #C(2845.531 -870.5343)
      #C(222.45422 1149.8602) #C(-3308.3872 917.76044)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5 8.24) #C(3.29 -8.93)
	      #C(34.12 -6.15) #C(49.27 -13.49) #C(32.5 42.73)
	      #C(-17.24 43.31) #C(-16.12 -8.25) #C(21.44 -49.08))))
	  (V2
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(32.5 42.73) #C(-17.24 43.31) #C(-16.12 -8.25)
	      #C(21.44 -49.08) #C(-39.66 -49.46)
	      #C(19.68 -5.55) #C(-8.82 25.37)
	      #C(-30.58 31.67))))
	  (SCALAR #C(-39.66 -49.46)))
      (GRID:COPY-TO (AXPY SCALAR V1 V2)))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(1808.3204d0 1422.3016000000002d0)
      #C(-589.3992d0 234.75039999999998d0)
      #C(-1673.4981999999998d0 -1451.9162000000001d0)
      #C(-2599.8236d0 -1950.9608000000003d0)
      #C(784.8158000000002d0 -3351.5818d0)
      #C(2845.5309999999995d0 -870.5342d0)
      #C(222.45420000000001d0 1149.8601999999998d0)
      #C(-3308.3871999999997d0 917.7603999999995d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0)
	      #C(34.12d0 -6.15d0) #C(49.27d0 -13.49d0)
	      #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)
	      #C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0))))
	  (V2
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)
	      #C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0)
	      #C(-39.66d0 -49.46d0) #C(19.68d0 -5.55d0)
	      #C(-8.82d0 25.37d0) #C(-30.58d0 31.67d0))))
	  (SCALAR #C(-39.66d0 -49.46d0)))
      (GRID:COPY-TO (AXPY SCALAR V1 V2))))))
