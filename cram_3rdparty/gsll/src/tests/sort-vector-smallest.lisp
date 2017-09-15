;; Regression test SORT-VECTOR-SMALLEST for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST SORT-VECTOR-SMALLEST
    (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
     (LIST #(-34.5f0 -13.49f0 -8.93f0))
     (MULTIPLE-VALUE-LIST
	 (LET ((V1
		(GRID:MAKE-FOREIGN-ARRAY
		 'SINGLE-FLOAT
		 :INITIAL-CONTENTS
		 '(-34.5f0 8.24f0 3.29f0 -8.93f0 34.12f0 -6.15f0 49.27f0 -13.49f0)))
	       (V2
		(GRID:MAKE-FOREIGN-ARRAY
		 'SINGLE-FLOAT
		 :INITIAL-CONTENTS '(32.5f0 42.73f0 -17.24f0))))
	   (GRID:COPY-TO (SORT-VECTOR-SMALLEST V2 V1) 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(-34.5d0 -13.49d0 -8.93d0))
   (MULTIPLE-VALUE-LIST
       (LET ((V1
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(-34.5d0 8.24d0 3.29d0 -8.93d0
					 34.12d0 -6.15d0 49.27d0
					 -13.49d0)))
	     (V2
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(32.5d0 42.73d0 -17.24d0))))
	 (GRID:COPY-TO (SORT-VECTOR-SMALLEST V2 V1) 'array 'double-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-91 -68 -64))
				     (MULTIPLE-VALUE-LIST
					 (LET ((V1
						(GRID:MAKE-FOREIGN-ARRAY
						 '(SIGNED-BYTE
						   8)
						 :INITIAL-CONTENTS
						 '(-64 -68 71
						   -91 52 -10
						   73 -5)))
					       (V2
						(GRID:MAKE-FOREIGN-ARRAY
						 '(SIGNED-BYTE
						   8)
						 :INITIAL-CONTENTS
						 '(123 32
						   28))))
					   (GRID:COPY-TO
					    (SORT-VECTOR-SMALLEST
					     V2 V1)
					     'array '(SIGNED-BYTE 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(44 67 116))
				     (MULTIPLE-VALUE-LIST
					 (LET ((V1
						(GRID:MAKE-FOREIGN-ARRAY
						 '(UNSIGNED-BYTE
						   8)
						 :INITIAL-CONTENTS
						 '(67 44 189
						   116 163
						   140 161
						   215)))
					       (V2
						(GRID:MAKE-FOREIGN-ARRAY
						 '(UNSIGNED-BYTE
						   8)
						 :INITIAL-CONTENTS
						 '(98 28
						   10))))
					   (GRID:COPY-TO
					    (SORT-VECTOR-SMALLEST
					     V2 V1)
					     'array '(unSIGNED-BYTE 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-91 -68 -64))
				     (MULTIPLE-VALUE-LIST
					 (LET ((V1
						(GRID:MAKE-FOREIGN-ARRAY
						 '(SIGNED-BYTE
						   16)
						 :INITIAL-CONTENTS
						 '(-64 -68 71
						   -91 52 -10
						   73 -5)))
					       (V2
						(GRID:MAKE-FOREIGN-ARRAY
						 '(SIGNED-BYTE
						   16)
						 :INITIAL-CONTENTS
						 '(123 32
						   28))))
					   (GRID:COPY-TO
					    (SORT-VECTOR-SMALLEST
					     V2 V1)
					     'array '(SIGNED-BYTE 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(44 67 116))
				     (MULTIPLE-VALUE-LIST
					 (LET ((V1
						(GRID:MAKE-FOREIGN-ARRAY
						 '(UNSIGNED-BYTE
						   16)
						 :INITIAL-CONTENTS
						 '(67 44 189
						   116 163
						   140 161
						   215)))
					       (V2
						(GRID:MAKE-FOREIGN-ARRAY
						 '(UNSIGNED-BYTE
						   16)
						 :INITIAL-CONTENTS
						 '(98 28
						   10))))
					   (GRID:COPY-TO
					    (SORT-VECTOR-SMALLEST
					     V2 V1)
					     'array '(unSIGNED-BYTE 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-91 -68 -64))
				     (MULTIPLE-VALUE-LIST
					 (LET ((V1
						(GRID:MAKE-FOREIGN-ARRAY
						 '(SIGNED-BYTE
						   32)
						 :INITIAL-CONTENTS
						 '(-64 -68 71
						   -91 52 -10
						   73 -5)))
					       (V2
						(GRID:MAKE-FOREIGN-ARRAY
						 '(SIGNED-BYTE
						   32)
						 :INITIAL-CONTENTS
						 '(123 32
						   28))))
					   (GRID:COPY-TO
					    (SORT-VECTOR-SMALLEST
					     V2 V1)
					     'array '(SIGNED-BYTE 32)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(44 67 116))
				     (MULTIPLE-VALUE-LIST
					 (LET ((V1
						(GRID:MAKE-FOREIGN-ARRAY
						 '(UNSIGNED-BYTE
						   32)
						 :INITIAL-CONTENTS
						 '(67 44 189
						   116 163
						   140 161
						   215)))
					       (V2
						(GRID:MAKE-FOREIGN-ARRAY
						 '(UNSIGNED-BYTE
						   32)
						 :INITIAL-CONTENTS
						 '(98 28
						   10))))
					   (GRID:COPY-TO
					    (SORT-VECTOR-SMALLEST
					     V2 V1)
					     'array '(unSIGNED-BYTE 32)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-91 -68 -64))
				     (MULTIPLE-VALUE-LIST
					 (LET ((V1
						(GRID:MAKE-FOREIGN-ARRAY
						 '(SIGNED-BYTE
						   64)
						 :INITIAL-CONTENTS
						 '(-64 -68 71
						   -91 52 -10
						   73 -5)))
					       (V2
						(GRID:MAKE-FOREIGN-ARRAY
						 '(SIGNED-BYTE
						   64)
						 :INITIAL-CONTENTS
						 '(123 32
						   28))))
					   (GRID:COPY-TO
					    (SORT-VECTOR-SMALLEST
					     V2 V1)
					     'array '(SIGNED-BYTE 64)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(44 67 116))
				     (MULTIPLE-VALUE-LIST
					 (LET ((V1
						(GRID:MAKE-FOREIGN-ARRAY
						 '(UNSIGNED-BYTE
						   64)
						 :INITIAL-CONTENTS
						 '(67 44 189
						   116 163
						   140 161
						   215)))
					       (V2
						(GRID:MAKE-FOREIGN-ARRAY
						 '(UNSIGNED-BYTE
						   64)
						 :INITIAL-CONTENTS
						 '(98 28
						   10))))
					   (GRID:COPY-TO
					    (SORT-VECTOR-SMALLEST
					     V2 V1)
					     'array '(unSIGNED-BYTE 64))))))

