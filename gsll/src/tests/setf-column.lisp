;; Regression test SETF-COLUMN for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST SETF-COLUMN
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-34.5f0 8.24f0 42.73f0)
	(-8.93f0 34.12f0 -17.24f0)
	(49.27f0 -13.49f0 43.31f0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS
	    '((-34.5f0 8.24f0 3.29f0)
	      (-8.93f0 34.12f0 -6.15f0)
	      (49.27f0 -13.49f0 32.5f0))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT :INITIAL-CONTENTS
	    '(42.73f0 -17.24f0 43.31f0))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-34.5d0 8.24d0 42.73d0)
	(-8.93d0 34.12d0 -17.24d0)
	(49.27d0 -13.49d0 43.31d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS
	    '((-34.5d0 8.24d0 3.29d0)
	      (-8.93d0 34.12d0 -6.15d0)
	      (49.27d0 -13.49d0 32.5d0))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT :INITIAL-CONTENTS
	    '(42.73d0 -17.24d0 43.31d0))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array 'double-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(42.73f0 -17.24f0))
	(#C(-8.93f0 34.12f0) #C(-6.15f0 49.27f0) #C(43.31f0 -16.12f0))
	(#C(49.27f0 -13.49f0) #C(32.5f0 42.73f0) #C(-8.25f0 21.44f0))))
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
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(42.73f0 -17.24f0) #C(43.31f0 -16.12f0) #C(-8.25f0 21.44f0)))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array '(complex single-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(42.73d0 -17.24d0))
	(#C(-8.93d0 34.12d0) #C(-6.15d0 49.27d0) #C(43.31d0 -16.12d0))
	(#C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-8.25d0 21.44d0))))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0))
	      (#C(-8.93d0 34.12d0) #C(-6.15d0 49.27d0) #C(-13.49d0 32.5d0))
	      (#C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(42.73d0 -17.24d0) #C(43.31d0 -16.12d0) #C(-8.25d0 21.44d0)))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array '(complex double-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-64 -68 32) (-91 52 28) (73 -5 30)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 8)
	    :INITIAL-CONTENTS '((-64 -68 71) (-91 52 -10) (73 -5 123))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 8)
	    :INITIAL-CONTENTS '(32 28 30))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array '(SIGNED-BYTE 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((67 44 28) (116 163 10) (161 215 19)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 8)
	    :INITIAL-CONTENTS '((67 44 189) (116 163 140) (161 215 98))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 8)
	    :INITIAL-CONTENTS '(28 10 19))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array '(UNSIGNED-BYTE 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-64 -68 32) (-91 52 28) (73 -5 30)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 16)
	    :INITIAL-CONTENTS
	    '((-64 -68 71) (-91 52 -10) (73 -5 123))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 16)
	    :INITIAL-CONTENTS '(32 28 30))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array '(SIGNED-BYTE 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((67 44 28) (116 163 10) (161 215 19)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 16)
	    :INITIAL-CONTENTS '((67 44 189) (116 163 140) (161 215 98))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 16) :INITIAL-CONTENTS '(28 10 19))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array '(unsigned-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-64 -68 32) (-91 52 28) (73 -5 30)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 32)
	    :INITIAL-CONTENTS '((-64 -68 71) (-91 52 -10) (73 -5 123))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 32)
	    :INITIAL-CONTENTS '(32 28 30))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array '(SIGNED-BYTE 32)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((67 44 28) (116 163 10) (161 215 19)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 32)
	    :INITIAL-CONTENTS
	    '((67 44 189) (116 163 140) (161 215 98))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 32)
	    :INITIAL-CONTENTS '(28 10 19))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array '(unsigned-byte 32)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-64 -68 32) (-91 52 28) (73 -5 30)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 64)
	    :INITIAL-CONTENTS
	    '((-64 -68 71) (-91 52 -10) (73 -5 123))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 64) :INITIAL-CONTENTS '(32 28 30))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array '(SIGNED-BYTE 64)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((67 44 28) (116 163 10) (161 215 19)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 64)
	    :INITIAL-CONTENTS '((67 44 189) (116 163 140) (161 215 98))))
	  (COL
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 64) :INITIAL-CONTENTS '(28 10 19))))
      (SETF (COLUMN M1 2) COL)
      (GRID:COPY-TO M1 'array '(unsigned-byte 64))))))

