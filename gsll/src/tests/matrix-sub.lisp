;; Regression test MATRIX-SUB for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MATRIX-SUB
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-77.229996f0 25.48f0 -40.02f0)
	(7.1900005f0 42.37f0 -27.59f0)
	(98.350006f0 26.17f0 81.96f0)))
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
	 (GRID:COPY-TO (ELT- M1 M2) 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-77.22999999999999d0 25.479999999999997d0
                              -40.02d0)
	(7.190000000000001d0 42.37d0
			     -27.590000000000003d0)
	(98.35d0 26.169999999999995d0
		 81.96000000000001d0)))
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
	 (GRID:COPY-TO (ELT- M1 M2) 'array 'double-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-96 -96 41) (-128 125 -2) (88 17 55)))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS
				       '((-64 -68 71) (-91 52 -10)
					 (73 -5 123))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS
				       '((32 28 30) (37 -73 -8)
					 (-15 -22 68)))))
	 (GRID:COPY-TO (ELT- M1 M2) 'array '(signed-byte 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((39 34 170) (88 241 179) (125 106 132)))
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
	 (GRID:COPY-TO (ELT- M1 M2) 'array '(unsigned-byte 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-96 -96 41) (-128 125 -2) (88 17 55)))
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
	 (GRID:COPY-TO (ELT- M1 M2) 'array '(signed-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((39 34 170) (88 65521 65459) (125 106 65412)))
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
	 (GRID:COPY-TO (ELT- M1 M2) 'array '(unsigned-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-96 -96 41) (-128 125 -2) (88 17 55)))
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
	 (GRID:COPY-TO (ELT- M1 M2) 'array '(signed-byte 32)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((39 34 170)
	(88 4294967281 4294967219)
	(125 106 4294967172)))
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
	 (GRID:COPY-TO (ELT- M1 M2) 'array '(unsigned-byte 32)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-96 -96 41) (-128 125 -2) (88 17 55)))
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
	 (GRID:COPY-TO (ELT- M1 M2) 'array '(signed-byte 64)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((39 34 170)
	(88 18446744073709551601 18446744073709551539)
	(125 106 18446744073709551492)))
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
	 (GRID:COPY-TO (ELT- M1 M2) 'array '(unsigned-byte 64))))))

