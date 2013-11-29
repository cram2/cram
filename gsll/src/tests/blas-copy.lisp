;; Regression test BLAS-COPY for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST BLAS-COPY
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-34.5 8.24 3.29))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS '(-34.5 8.24 3.29)))
	  (V2 (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :DIMENSIONS '3)))
      (BLAS-COPY V1 V2)
      (GRID:COPY-TO V2))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-34.5d0 8.24d0 3.29d0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS '(-34.5d0 8.24d0 3.29d0)))
	  (V2 (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS '3)))
      (BLAS-COPY V1 V2)
      (GRID:COPY-TO V2))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(#C(-34.5 8.24) #C(3.29 -8.93) #C(34.12 -6.15)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5 8.24) #C(3.29 -8.93) #C(34.12 -6.15))))
	  (V2
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT) :DIMENSIONS '3)))
      (BLAS-COPY V1 V2)
      (GRID:COPY-TO V2))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0)
      #C(34.12d0 -6.15d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0)
	      #C(34.12d0 -6.15d0))))
	  (V2
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT) :DIMENSIONS '3)))
      (BLAS-COPY V1 V2)
      (GRID:COPY-TO V2)))))
