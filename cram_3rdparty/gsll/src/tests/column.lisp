;; Regression test COLUMN for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST COLUMN
    (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
     (LIST #(8.24f0 34.12f0 -13.49f0))
     (MULTIPLE-VALUE-LIST
	 (LET ((M1
		(GRID:MAKE-FOREIGN-ARRAY
		 'SINGLE-FLOAT
		 :INITIAL-CONTENTS
		 '((-34.5f0 8.24f0 3.29f0)
		   (-8.93f0 34.12f0 -6.15f0)
		   (49.27f0 -13.49f0 32.5f0))))
	       (COL (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :DIMENSIONS '3)))
	   (GRID:COPY-TO (COLUMN M1 1 COL) 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(8.24d0 34.12d0 -13.49d0))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       'DOUBLE-FLOAT :INITIAL-CONTENTS
	       '((-34.5d0 8.24d0 3.29d0)
		 (-8.93d0 34.12d0 -6.15d0)
		 (49.27d0 -13.49d0 32.5d0))))
	     (COL (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array 'double-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(#C(3.29f0 -8.93f0) #C(-6.15f0 49.27f0) #C(32.5f0 42.73f0)))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX SINGLE-FLOAT)
	       :INITIAL-CONTENTS
	       '((#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(34.12f0 -6.15f0))
		 (#C(-8.93f0 34.12f0) #C(-6.15f0 49.27f0) #C(-13.49f0 32.5f0))
		 (#C(49.27f0 -13.49f0) #C(32.5f0 42.73f0) #C(-17.24f0 43.31f0)))))
	     (COL
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX SINGLE-FLOAT) :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array '(complex single-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(3.29d0 -8.93d0) #C(-6.15d0 49.27d0)
      #C(32.5d0 42.73d0)))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX DOUBLE-FLOAT)
	       :INITIAL-CONTENTS
	       '((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0))
		 (#C(-8.93d0 34.12d0) #C(-6.15d0 49.27d0) #C(-13.49d0 32.5d0))
		 (#C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)))))
	     (COL
	      (GRID:MAKE-FOREIGN-ARRAY '(COMPLEX DOUBLE-FLOAT) :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array '(complex double-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-68 52 -5))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(SIGNED-BYTE 8)
	       :INITIAL-CONTENTS
	       '((-64 -68 71)
		 (-91 52 -10)
		 (73 -5 123))))
	     (COL
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(SIGNED-BYTE 8)
	       :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array '(signed-byte 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(44 163 215))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(UNSIGNED-BYTE 8)
	       :INITIAL-CONTENTS
	       '((67 44 189)
		 (116 163 140)
		 (161 215 98))))
	     (COL
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(UNSIGNED-BYTE 8)
	       :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array '(unsigned-byte 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-68 52 -5))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(SIGNED-BYTE 16)
	       :INITIAL-CONTENTS
	       '((-64 -68 71)
		 (-91 52 -10)
		 (73 -5 123))))
	     (COL
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(SIGNED-BYTE 16) :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array '(signed-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(44 163 215))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(UNSIGNED-BYTE 16)
	       :INITIAL-CONTENTS
	       '((67 44 189)
		 (116 163 140)
		 (161 215 98))))
	     (COL
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(UNSIGNED-BYTE 16)
	       :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array '(unsigned-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-68 52 -5))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(SIGNED-BYTE 32)
	       :INITIAL-CONTENTS
	       '((-64 -68 71)
		 (-91 52 -10)
		 (73 -5 123))))
	     (COL
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(SIGNED-BYTE 32) :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array '(signed-byte 32)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(44 163 215))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(UNSIGNED-BYTE 32)
	       :INITIAL-CONTENTS
	       '((67 44 189)
		 (116 163 140)
		 (161 215 98))))
	     (COL
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(UNSIGNED-BYTE 32) :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array '(unsigned-byte 32)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-68 52 -5))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(SIGNED-BYTE 64)
	       :INITIAL-CONTENTS
	       '((-64 -68 71)
		 (-91 52 -10)
		 (73 -5 123))))
	     (COL
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(SIGNED-BYTE 64)
	       :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array '(signed-byte 64)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(44 163 215))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(UNSIGNED-BYTE 64)
	       :INITIAL-CONTENTS
	       '((67 44 189)
		 (116 163 140)
		 (161 215 98))))
	     (COL
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(UNSIGNED-BYTE 64)
	       :DIMENSIONS '3)))
	 (GRID:COPY-TO (COLUMN M1 1 COL) 'array '(unsigned-byte 64))))))

