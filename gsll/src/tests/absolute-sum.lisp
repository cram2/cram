;; Regression test ABSOLUTE-SUM for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST ABSOLUTE-SUM
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 157.99)
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT
	    :INITIAL-CONTENTS
	    '(-34.5 8.24 3.29 -8.93 34.12 -6.15 49.27 -13.49))))
      (ABSOLUTE-SUM V1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 157.99d0)
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT
	    :INITIAL-CONTENTS
	    '(-34.5d0 8.24d0 3.29d0 -8.93d0 34.12d0 -6.15d0 49.27d0 -13.49d0))))
      (ABSOLUTE-SUM V1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 388.65997)
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX
	      SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5 8.24) #C(3.29 -8.93) #C(34.12 -6.15) #C(49.27 -13.49)
	      #C(32.5 42.73) #C(-17.24 43.31) #C(-16.12 -8.25) #C(21.44 -49.08)))))
      (ABSOLUTE-SUM V1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 388.65999999999997d0)
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   #2m(#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0)
		 #C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)
		 #C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0))))
      (ABSOLUTE-SUM V1)))))
