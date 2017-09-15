;; Regression test MATRIX-MULT for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MATRIX-MULT
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-1474.1849f0 -142.05759f0 142.4899f0)
	(143.95161f0 -281.49f0 -131.856f0)
	(-2418.1716f0 535.01337f0 -1607.45f0)))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY
	       'SINGLE-FLOAT :INITIAL-CONTENTS
	       '((-34.5f0 8.24f0 3.29f0)
		 (-8.93f0 34.12f0 -6.15f0)
		 (49.27f0 -13.49f0 32.5f0))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY
	       'SINGLE-FLOAT :INITIAL-CONTENTS
	       '((42.73f0 -17.24f0 43.31f0)
		 (-16.12f0 -8.25f0 21.44f0)
		 (-49.08f0 -39.66f0 -49.46f0)))))
	 (GRID:COPY-TO (ELT* M1 M2) 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-1474.185d0 -142.05759999999998d0 142.4899d0)
	(143.9516d0 -281.48999999999995d0
		    -131.85600000000002d0)
	(-2418.1716d0 535.0133999999999d0 -1607.45d0)))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '((-34.5d0 8.24d0 3.29d0)
					 (-8.93d0 34.12d0 -6.15d0)
					 (49.27d0 -13.49d0 32.5d0))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '((42.73d0 -17.24d0 43.31d0)
					 (-16.12d0 -8.25d0 21.44d0)
					 (-49.08d0 -39.66d0 -49.46d0)))))
	 (GRID:COPY-TO (ELT* M1 M2) 'array 'double-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((0 -112 82) (-39 44 80) (-71 110 -84)))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS
				       '((-64 -68 71) (-91 52 -10)
					 (73 -5 123))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS
				       '((32 28 30) (37 -73 -8)
					 (-15 -22 68)))))
	 (GRID:COPY-TO (ELT* M1 M2) 'array '(signed-byte 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((84 184 7) (176 86 172) (164 139 252)))
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
	 (GRID:COPY-TO (ELT* M1 M2) 'array '(unsigned-byte 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-2048 -1904 2130)
	(-3367 -3796 80)
	(-1095 110 8364)))
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
	 (GRID:COPY-TO (ELT* M1 M2) 'array '(signed-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((1876 440 3591)
	(3248 29014 30380)
	(5796 23435 21756)))
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
	 (GRID:COPY-TO (ELT* M1 M2) 'array '(unsigned-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-2048 -1904 2130)
	(-3367 -3796 80)
	(-1095 110 8364)))
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
	 (GRID:COPY-TO (ELT* M1 M2) 'array '(signed-byte 32)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((1876 440 3591)
	(3248 29014 30380)
	(5796 23435 21756)))
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
	 (GRID:COPY-TO (ELT* M1 M2) 'array '(unsigned-byte 32)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-2048 -1904 2130)
	(-3367 -3796 80)
	(-1095 110 8364)))
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
	 (GRID:COPY-TO (ELT* M1 M2) 'array '(signed-byte 64)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((1876 440 3591)
	(3248 29014 30380)
	(5796 23435 21756)))
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
	 (GRID:COPY-TO (ELT* M1 M2) 'array '(unsigned-byte 64))))))

