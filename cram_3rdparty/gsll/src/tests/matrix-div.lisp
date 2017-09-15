;; Regression test MATRIX-DIV for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MATRIX-DIV
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-0.8073953f0 -0.47795823f0 0.075963974f0)
	(0.5539702f0 -4.1357574f0 -0.28684703f0)
	(-1.0038712f0 0.3401412f0 -0.6570967f0)))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
				       '((-34.5f0 8.24f0 3.29f0)
					 (-8.93f0 34.12f0 -6.15f0)
					 (49.27f0 -13.49f0 32.5f0))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
				       '((42.73f0 -17.24f0 43.31f0)
					 (-16.12f0 -8.25f0 21.44f0)
					 (-49.08f0 -39.66f0 -49.46f0)))))
	 (GRID:COPY-TO (ELT/ M1 M2) 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #2A((-0.8073952726421718d0 -0.4779582366589328d0
			       0.07596398060494113d0)
	(0.553970223325062d0 -4.135757575757576d0
			     -0.2868470149253731d0)
	(-1.0038712306438469d0 0.3401412002017146d0
			       -0.6570966437525273d0)))
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
	 (GRID:COPY-TO (ELT/ M1 M2) 'array 'double-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-2 -2 2) (-2 0 1) (-4 0 1)))
   (MULTIPLE-VALUE-LIST
       (LET ((M1
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS
				       '((-64 -68 71) (-91 52 -10)
					 (73 -5 123))))
	     (M2
	      (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS
				       '((32 28 30) (37 -73 -8)
					 (-15 -22 68)))))
	 (GRID:COPY-TO (ELT/ M1 M2) 'array '(signed-byte 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((2 4 9) (4 0 0) (4 1 0)))
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
	 (GRID:COPY-TO (ELT/ M1 M2) 'array '(unsigned-byte 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-2 -2 2) (-2 0 1) (-4 0 1)))
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
	 (GRID:COPY-TO (ELT/ M1 M2) 'array '(signed-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((2 4 9) (4 0 0) (4 1 0)))
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
	 (GRID:COPY-TO (ELT/ M1 M2) 'array '(unsigned-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-2 -2 2) (-2 0 1) (-4 0 1)))
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
	 (GRID:COPY-TO (ELT/ M1 M2) 'array '(signed-byte 32)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((2 4 9) (4 0 0) (4 1 0)))
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
	 (GRID:COPY-TO (ELT/ M1 M2) 'array '(unsigned-byte 32)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((-2 -2 2) (-2 0 1) (-4 0 1)))
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
	 (GRID:COPY-TO (ELT/ M1 M2) 'array '(signed-byte 64)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #2A((2 4 9) (4 0 0) (4 1 0)))
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
	 (GRID:COPY-TO (ELT/ M1 M2) 'array '(unsigned-byte 64))))))

