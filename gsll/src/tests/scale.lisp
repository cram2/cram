;; Regression test SCALE for GSLL, automatically generated
;;
;; Copyright 2009, 2010 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST SCALE
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-1121.25 267.8) #C(106.924995 -290.225)
      #C(1108.9 -199.875) #C(1601.275 -438.425)
      #C(1056.25 1388.725) #C(-560.3 1407.5751)
      #C(-523.9 -268.125) #C(696.8 -1595.1001)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5 8.24) #C(3.29 -8.93) #C(34.12 -6.15)
	      #C(49.27 -13.49) #C(32.5 42.73)
	      #C(-17.24 43.31) #C(-16.12 -8.25) #C(21.44 -49.08))))
	  (SCALAR 32.5))
      (GRID:COPY-TO (SCALE SCALAR V1)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-1121.25d0 267.8d0)
      #C(106.925d0 -290.22499999999997d0)
      #C(1108.8999999999999d0 -199.875d0)
      #C(1601.275d0 -438.425d0) #C(1056.25d0 1388.725d0)
      #C(-560.3d0 1407.575d0) #C(-523.9d0 -268.125d0)
      #C(696.8000000000001d0 -1595.1d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0)
	      #C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)
	      #C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0))))
	  (SCALAR 32.5d0))
      (GRID:COPY-TO (SCALE SCALAR V1)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-1121.25 267.8 106.924995 -290.225 1108.9 -199.875 1601.275 -438.425))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT
	    :INITIAL-CONTENTS
	    '(-34.5 8.24 3.29 -8.93 34.12 -6.15 49.27 -13.49)))
	  (SCALAR 32.5))
      (GRID:COPY-TO (SCALE SCALAR V1)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(-1121.25d0 267.8d0 106.925d0 -290.22499999999997d0
      1108.8999999999999d0 -199.875d0 1601.275d0
      -438.425d0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS
	    '(-34.5d0 8.24d0 3.29d0 -8.93d0
	      34.12d0 -6.15d0 49.27d0 -13.49d0)))
	  (SCALAR 32.5d0))
      (GRID:COPY-TO (SCALE SCALAR V1)))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-1473.3452 -1206.385) #C(488.5039 -149.64331)
      #C(1371.6895 1258.0725) #C(2177.7026 1666.8821)
      #C(-769.6029 2777.45) #C(-2410.9363 670.9099)
      #C(-171.37753 -956.9326) #C(2793.9885 -678.9689)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5 8.24) #C(3.29 -8.93) #C(34.12 -6.15)
	      #C(49.27 -13.49) #C(32.5 42.73)
	      #C(-17.24 43.31) #C(-16.12 -8.25) #C(21.44 -49.08))))
	  (SCALAR #C(32.5 42.73)))
      (GRID:COPY-TO (SCALE SCALAR V1)))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-1473.3452d0 -1206.385d0)
      #C(488.5039d0 -149.64329999999998d0)
      #C(1371.6895d0 1258.0725999999997d0)
      #C(2177.7027d0 1666.8821d0)
      #C(-769.6028999999996d0 2777.45d0)
      #C(-2410.9363d0 670.9098000000001d0)
      #C(-171.3775d0 -956.9326d0)
      #C(2793.9883999999997d0 -678.9687999999999d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0)
	      #C(34.12d0 -6.15d0) #C(49.27d0 -13.49d0)
	      #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)
	      #C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0))))
	  (SCALAR #C(32.5d0 42.73d0)))
      (GRID:COPY-TO (SCALE SCALAR V1))))))
