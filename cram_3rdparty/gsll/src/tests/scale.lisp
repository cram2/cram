;; Regression test SCALE for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST SCALE
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-1121.25f0 267.8f0) #C(106.924995f0 -290.225f0)
      #C(1108.9f0 -199.875f0) #C(1601.275f0 -438.425f0)
      #C(1056.25f0 1388.725f0) #C(-560.3f0 1407.5751f0)
      #C(-523.9f0 -268.125f0) #C(696.8f0 -1595.1001f0)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(34.12f0 -6.15f0)
	      #C(49.27f0 -13.49f0) #C(32.5f0 42.73f0)
	      #C(-17.24f0 43.31f0) #C(-16.12f0 -8.25f0) #C(21.44f0 -49.08f0))))
	  (SCALAR 32.5f0))
      (GRID:COPY-TO (SCALE SCALAR V1) 'array '(complex single-float)))))
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
      (GRID:COPY-TO (SCALE SCALAR V1) 'array '(complex double-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-1121.25f0 267.8f0 106.924995f0 -290.225f0 1108.9f0 -199.875f0 1601.275f0 -438.425f0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT
	    :INITIAL-CONTENTS
	    '(-34.5f0 8.24f0 3.29f0 -8.93f0 34.12f0 -6.15f0 49.27f0 -13.49f0)))
	  (SCALAR 32.5f0))
      (GRID:COPY-TO (SCALE SCALAR V1) 'array 'single-float))))
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
      (GRID:COPY-TO (SCALE SCALAR V1) 'array 'double-float))))
  #+fsbv
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-1473.3452f0 -1206.385f0) #C(488.5039f0 -149.64331f0)
      #C(1371.6895f0 1258.0725f0) #C(2177.7026f0 1666.8821f0)
      #C(-769.6029f0 2777.45f0) #C(-2410.9363f0 670.9099f0)
      #C(-171.37753f0 -956.9326f0) #C(2793.9885f0 -678.9689f0)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(34.12f0 -6.15f0)
	      #C(49.27f0 -13.49f0) #C(32.5f0 42.73f0)
	      #C(-17.24f0 43.31f0) #C(-16.12f0 -8.25f0) #C(21.44f0 -49.08f0))))
	  (SCALAR #C(32.5f0 42.73f0)))
      (GRID:COPY-TO (SCALE SCALAR V1) 'array '(complex single-float)))))
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
      (GRID:COPY-TO (SCALE SCALAR V1) 'array '(complex double-float))))))
