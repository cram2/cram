;; Regression test CDOT for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST CDOT
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #C(-1.1684039224427728d28 0.0d0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5 8.24) #C(3.29 -8.93) #C(34.12 -6.15)
	      #C(49.27 -13.49) #C(32.5 42.73)
	      #C(-17.24 43.31) #C(-16.12 -8.25) #C(21.44 -49.08))))
	  (V2
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(32.5 42.73) #C(-17.24 43.31) #C(-16.12 -8.25)
	      #C(21.44 -49.08) #C(-39.66 -49.46)
	      #C(19.68 -5.55) #C(-8.82 25.37) #C(-30.58 31.67)))))
      (CDOT V1 V2))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #C(-6252.624d0 -6236.050300000001d0))
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
	      #C(-39.66d0 -49.46d0)
	      #C(19.68d0 -5.55d0) #C(-8.82d0 25.37d0)
	      #C(-30.58d0 31.67d0)))))
      (CDOT V1 V2)))))

