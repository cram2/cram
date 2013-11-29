;; Regression test SETF-ROW for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST SETF-ROW
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-34.5 8.24 3.29)
	(-8.93 34.12 -6.15)
	(42.73 -17.24 43.31)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT
	    :INITIAL-CONTENTS
	    '((-34.5 8.24 3.29)
	      (-8.93 34.12 -6.15)
	      (49.27 -13.49 32.5))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY
	    'SINGLE-FLOAT
	    :INITIAL-CONTENTS '(42.73 -17.24 43.31))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-34.5d0 8.24d0 3.29d0)
	(-8.93d0 34.12d0 -6.15d0)
	(42.73d0 -17.24d0 43.31d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT
	    :INITIAL-CONTENTS
	    '((-34.5d0 8.24d0 3.29d0)
	      (-8.93d0 34.12d0 -6.15d0)
	      (49.27d0 -13.49d0 32.5d0))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY
	    'DOUBLE-FLOAT
	    :INITIAL-CONTENTS '(42.73d0 -17.24d0 43.31d0))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(-34.5 8.24) #C(3.29 -8.93) #C(34.12 -6.15))
	(#C(-8.93 34.12) #C(-6.15 49.27) #C(-13.49 32.5))
	(#C(42.73 -17.24) #C(43.31 -16.12)
	   #C(-8.25 21.44))))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(-34.5 8.24) #C(3.29 -8.93) #C(34.12 -6.15))
	      (#C(-8.93 34.12) #C(-6.15 49.27) #C(-13.49 32.5))
	      (#C(49.27 -13.49) #C(32.5 42.73) #C(-17.24 43.31)))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS '(#C(42.73 -17.24) #C(43.31 -16.12) #C(-8.25 21.44)))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0))
	(#C(-8.93d0 34.12d0) #C(-6.15d0 49.27d0) #C(-13.49d0 32.5d0))
	(#C(42.73d0 -17.24d0) #C(43.31d0 -16.12d0) #C(-8.25d0 21.44d0))))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0))
	      (#C(-8.93d0 34.12d0) #C(-6.15d0 49.27d0) #C(-13.49d0 32.5d0))
	      (#C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(42.73d0 -17.24d0) #C(43.31d0 -16.12d0) #C(-8.25d0 21.44d0)))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-64 -68 71) (-91 52 -10) (32 28 30)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 8)
	    :INITIAL-CONTENTS
	    '((-64 -68 71) (-91 52 -10) (73 -5 123))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 8)
	    :INITIAL-CONTENTS '(32 28 30))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((67 44 189) (116 163 140) (28 10 19)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 8)
	    :INITIAL-CONTENTS
	    '((67 44 189) (116 163 140) (161 215 98))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 8)
	    :INITIAL-CONTENTS '(28 10 19))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-64 -68 71) (-91 52 -10) (32 28 30)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 16)
	    :INITIAL-CONTENTS
	    '((-64 -68 71) (-91 52 -10) (73 -5 123))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 16)
	    :INITIAL-CONTENTS '(32 28 30))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((67 44 189) (116 163 140) (28 10 19)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 16)
	    :INITIAL-CONTENTS '((67 44 189) (116 163 140) (161 215 98))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 16)
	    :INITIAL-CONTENTS '(28 10 19))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-64 -68 71) (-91 52 -10) (32 28 30)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 32)
	    :INITIAL-CONTENTS
	    '((-64 -68 71) (-91 52 -10) (73 -5 123))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 32) :INITIAL-CONTENTS '(32 28 30))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((67 44 189) (116 163 140) (28 10 19)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 32)
	    :INITIAL-CONTENTS
	    '((67 44 189) (116 163 140) (161 215 98))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 32)
	    :INITIAL-CONTENTS '(28 10 19))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-64 -68 71) (-91 52 -10) (32 28 30)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 64)
	    :INITIAL-CONTENTS
	    '((-64 -68 71) (-91 52 -10) (73 -5 123))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(SIGNED-BYTE 64)
	    :INITIAL-CONTENTS '(32 28 30))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((67 44 189) (116 163 140) (28 10 19)))
   (MULTIPLE-VALUE-LIST
    (LET ((M1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 64)
	    :INITIAL-CONTENTS
	    '((67 44 189) (116 163 140) (161 215 98))))
	  (ROW
	   (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 64)
				    :INITIAL-CONTENTS '(28 10 19))))
      (SETF (ROW M1 2) ROW)
      (GRID:COPY-TO M1)))))

