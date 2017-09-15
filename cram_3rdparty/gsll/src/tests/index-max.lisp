;; Regression test INDEX-MAX for GSLL, automatically generated
;;
;; Copyright 2009, 2010, 2011 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST INDEX-MAX
    (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
     (LIST 6)
     (MULTIPLE-VALUE-LIST
	 (LET ((V1
		(GRID:MAKE-FOREIGN-ARRAY
		 'SINGLE-FLOAT
		 :INITIAL-CONTENTS
		 '(-34.5f0 8.24f0 3.29f0 -8.93f0 34.12f0 -6.15f0 49.27f0 -13.49f0))))
	   (INDEX-MAX V1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 6)
   (MULTIPLE-VALUE-LIST
       (LET ((V1
	      (GRID:MAKE-FOREIGN-ARRAY
	       'DOUBLE-FLOAT
	       :INITIAL-CONTENTS
	       '(-34.5d0 8.24d0 3.29d0 -8.93d0 34.12d0 -6.15d0 49.27d0 -13.49d0))))
	 (INDEX-MAX V1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 4)
   (MULTIPLE-VALUE-LIST
       (LET ((V1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX SINGLE-FLOAT)
	       :INITIAL-CONTENTS
	       '(#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(34.12f0 -6.15f0)
		 #C(49.27f0 -13.49f0) #C(32.5f0 42.73f0)
		 #C(-17.24f0 43.31f0) #C(-16.12f0 -8.25f0) #C(21.44f0 -49.08f0)))))
	 (INDEX-MAX V1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 4)
   (MULTIPLE-VALUE-LIST
       (LET ((V1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX DOUBLE-FLOAT)
	       :INITIAL-CONTENTS
	       '(#C(-34.5d0 8.24d0)
		 #C(3.29d0 -8.93d0)
		 #C(34.12d0 -6.15d0)
		 #C(49.27d0 -13.49d0)
		 #C(32.5d0 42.73d0)
		 #C(-17.24d0 43.31d0)
		 #C(-16.12d0 -8.25d0)
		 #C(21.44d0 -49.08d0)))))
	 (INDEX-MAX V1)))))
