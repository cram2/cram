;; Regression test MATRIX-SWAP for GSLL, automatically generated
;;
;; Copyright 2009, 2010, 2014 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST MATRIX-SWAP
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST
     #2A((42.73f0 -17.24f0 43.31f0)
	 (-16.12f0 -8.25f0 21.44f0)
	 (-49.08f0 -39.66f0 -49.46f0))
     #2A((-34.5f0 8.24f0 3.29f0)
	 (-8.93f0 34.12f0 -6.15f0)
	 (49.27f0 -13.49f0 32.5f0))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       'SINGLE-FLOAT
	       :INITIAL-CONTENTS
	       '((-34.5f0 8.24f0 3.29f0)
		 (-8.93f0 34.12f0 -6.15f0)
		 (49.27f0 -13.49f0 32.5f0))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY
	       'SINGLE-FLOAT
	       :INITIAL-CONTENTS
	       '((42.73f0 -17.24f0 43.31f0)
		 (-16.12f0 -8.25f0 21.44f0)
		 (-49.08f0 -39.66f0 -49.46f0)))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array 'single-float) (GRID:COPY-TO M2 'array 'single-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST
     #2A((42.73d0 -17.24d0 43.31d0)
	 (-16.12d0 -8.25d0 21.44d0)
	 (-49.08d0 -39.66d0 -49.46d0))
     #2A((-34.5d0 8.24d0 3.29d0)
	 (-8.93d0 34.12d0 -6.15d0)
	 (49.27d0 -13.49d0 32.5d0))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       'DOUBLE-FLOAT :INITIAL-CONTENTS
	       '((-34.5d0 8.24d0 3.29d0)
		 (-8.93d0 34.12d0 -6.15d0)
		 (49.27d0 -13.49d0 32.5d0))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY
	       'DOUBLE-FLOAT :INITIAL-CONTENTS
	       '((42.73d0 -17.24d0 43.31d0)
		 (-16.12d0 -8.25d0 21.44d0)
		 (-49.08d0 -39.66d0 -49.46d0)))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array 'double-float) (GRID:COPY-TO M2 'array 'double-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST
     #2A((#C(42.73f0 -17.24f0) #C(43.31f0 -16.12f0) #C(-8.25f0 21.44f0))
	 (#C(-16.12f0 -8.25f0) #C(21.44f0 -49.08f0) #C(-39.66f0 -49.46f0))
	 (#C(-49.08f0 -39.66f0) #C(-49.46f0 19.68f0) #C(-5.55f0 -8.82f0)))
     #2A((#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(34.12f0 -6.15f0))
	 (#C(-8.93f0 34.12f0) #C(-6.15f0 49.27f0) #C(-13.49f0 32.5f0))
	 (#C(49.27f0 -13.49f0) #C(32.5f0 42.73f0) #C(-17.24f0 43.31f0)))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX SINGLE-FLOAT)
	       :INITIAL-CONTENTS
	       '((#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(34.12f0 -6.15f0))
		 (#C(-8.93f0 34.12f0) #C(-6.15f0 49.27f0) #C(-13.49f0 32.5f0))
		 (#C(49.27f0 -13.49f0) #C(32.5f0 42.73f0) #C(-17.24f0 43.31f0)))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX SINGLE-FLOAT)
	       :INITIAL-CONTENTS
	       '((#C(42.73f0 -17.24f0) #C(43.31f0 -16.12f0) #C(-8.25f0 21.44f0))
		 (#C(-16.12f0 -8.25f0) #C(21.44f0 -49.08f0) #C(-39.66f0 -49.46f0))
		 (#C(-49.08f0 -39.66f0) #C(-49.46f0 19.68f0) #C(-5.55f0 -8.82f0))))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array '(complex single-float)) (GRID:COPY-TO M2 'array '(complex single-float))))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST
     #2A((#C(42.73d0 -17.24d0) #C(43.31d0 -16.12d0) #C(-8.25d0 21.44d0))
	 (#C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0) #C(-39.66d0 -49.46d0))
	 (#C(-49.08d0 -39.66d0) #C(-49.46d0 19.68d0) #C(-5.55d0 -8.82d0)))
     #2A((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0))
	 (#C(-8.93d0 34.12d0) #C(-6.15d0 49.27d0) #C(-13.49d0 32.5d0))
	 (#C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX DOUBLE-FLOAT)
	       :INITIAL-CONTENTS
	       '((#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0))
		 (#C(-8.93d0 34.12d0) #C(-6.15d0 49.27d0) #C(-13.49d0 32.5d0))
		 (#C(49.27d0 -13.49d0) #C(32.5d0 42.73d0) #C(-17.24d0 43.31d0)))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY
	       '(COMPLEX DOUBLE-FLOAT)
	       :INITIAL-CONTENTS
	       '((#C(42.73d0 -17.24d0) #C(43.31d0 -16.12d0) #C(-8.25d0 21.44d0))
		 (#C(-16.12d0 -8.25d0) #C(21.44d0 -49.08d0) #C(-39.66d0 -49.46d0))
		 (#C(-49.08d0 -39.66d0) #C(-49.46d0 19.68d0) #C(-5.55d0 -8.82d0))))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array '(complex double-float)) (GRID:COPY-TO M2 'array '(complex double-float))))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST #2A((32 28 30) (37 -73 -8) (-15 -22 68))
	  #2A((-64 -68 71) (-91 52 -10) (73 -5 123))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS
				       '((-64 -68 71) (-91 52 -10)
					 (73 -5 123))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS
				       '((32 28 30) (37 -73 -8)
					 (-15 -22 68)))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array '(signed-byte 8)) (GRID:COPY-TO M2 'array '(signed-byte 8))))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST #2A((28 10 19) (28 178 217) (36 109 222))
	  #2A((67 44 189) (116 163 140) (161 215 98))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 8)
				       :INITIAL-CONTENTS
				       '((67 44 189) (116 163 140)
					 (161 215 98))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 8)
				       :INITIAL-CONTENTS
				       '((28 10 19) (28 178 217)
					 (36 109 222)))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array '(unsigned-byte 8)) (GRID:COPY-TO M2 'array '(unsigned-byte 8))))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST #2A((32 28 30) (37 -73 -8) (-15 -22 68))
	  #2A((-64 -68 71) (-91 52 -10) (73 -5 123))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 16)
				       :INITIAL-CONTENTS
				       '((-64 -68 71) (-91 52 -10)
					 (73 -5 123))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 16)
				       :INITIAL-CONTENTS
				       '((32 28 30) (37 -73 -8)
					 (-15 -22 68)))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array '(signed-byte 16)) (GRID:COPY-TO M2 'array '(signed-byte 16))))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST #2A((28 10 19) (28 178 217) (36 109 222))
	  #2A((67 44 189) (116 163 140) (161 215 98))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 16)
				       :INITIAL-CONTENTS
				       '((67 44 189) (116 163 140)
					 (161 215 98))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 16)
				       :INITIAL-CONTENTS
				       '((28 10 19) (28 178 217)
					 (36 109 222)))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array '(unsigned-byte 16)) (GRID:COPY-TO M2 'array '(unsigned-byte 16))))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST #2A((32 28 30) (37 -73 -8) (-15 -22 68))
	  #2A((-64 -68 71) (-91 52 -10) (73 -5 123))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 32)
				       :INITIAL-CONTENTS
				       '((-64 -68 71) (-91 52 -10)
					 (73 -5 123))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 32)
				       :INITIAL-CONTENTS
				       '((32 28 30) (37 -73 -8)
					 (-15 -22 68)))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array '(signed-byte 32)) (GRID:COPY-TO M2 'array '(signed-byte 32))))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST #2A((28 10 19) (28 178 217) (36 109 222))
	  #2A((67 44 189) (116 163 140) (161 215 98))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 32)
				       :INITIAL-CONTENTS
				       '((67 44 189) (116 163 140)
					 (161 215 98))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 32)
				       :INITIAL-CONTENTS
				       '((28 10 19) (28 178 217)
					 (36 109 222)))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array '(unsigned-byte 32)) (GRID:COPY-TO M2 'array '(unsigned-byte 32))))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST #2A((32 28 30) (37 -73 -8) (-15 -22 68))
	  #2A((-64 -68 71) (-91 52 -10) (73 -5 123))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 64)
				       :INITIAL-CONTENTS
				       '((-64 -68 71) (-91 52 -10)
					 (73 -5 123))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 64)
				       :INITIAL-CONTENTS
				       '((32 28 30) (37 -73 -8)
					 (-15 -22 68)))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array '(signed-byte 64)) (GRID:COPY-TO M2 'array '(signed-byte 64))))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST #2A((28 10 19) (28 178 217) (36 109 222))
	  #2A((67 44 189) (116 163 140) (161 215 98))))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 64)
				       :INITIAL-CONTENTS
				       '((67 44 189) (116 163 140)
					 (161 215 98))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 64)
				       :INITIAL-CONTENTS
				       '((28 10 19) (28 178 217)
					 (36 109 222)))))
	 (SWAP M2 M1)
	 (LIST (GRID:COPY-TO M1 'array '(unsigned-byte 64)) (GRID:COPY-TO M2 'array '(unsigned-byte 64)))))))

