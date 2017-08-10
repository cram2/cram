;; Regression test VECTOR-SET-ALL for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST VECTOR-SET-ALL
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST #(-34.5f0 -34.5f0 -34.5f0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1 (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :DIMENSIONS '3)))
      (GRID:COPY-TO (SET-ALL V1 -34.5f0) 'array 'single-float))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST #(-34.5d0 -34.5d0 -34.5d0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1 (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS '3)))
      (GRID:COPY-TO (SET-ALL V1 -34.5d0) 'array 'double-float))))
  #+fsbv
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST #(#C(-34.5f0 8.24f0) #C(-34.5f0 8.24f0) #C(-34.5f0 8.24f0)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY '(COMPLEX SINGLE-FLOAT)
			:DIMENSIONS '3)))
      (GRID:COPY-TO (SET-ALL V1 #C(-34.5f0 8.24f0)) 'array '(complex single-float)))))
  #+fsbv
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(-34.5d0 8.24d0) #C(-34.5d0 8.24d0)
      #C(-34.5d0 8.24d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY '(COMPLEX DOUBLE-FLOAT)
			:DIMENSIONS '3)))
      (GRID:COPY-TO (SET-ALL V1 #C(-34.5d0 8.24d0)) 'array '(complex double-float)))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL (LIST #(-64 -64 -64))
				    (MULTIPLE-VALUE-LIST
				     (LET ((V1
					    (GRID:MAKE-FOREIGN-ARRAY
					     '(SIGNED-BYTE
					       8)
					     :DIMENSIONS
					     '3)))
				       (GRID:COPY-TO
					(SET-ALL V1
						 -64)
					 'array '(signed-byte 8)))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL (LIST #(67 67 67))
				    (MULTIPLE-VALUE-LIST
				     (LET ((V1
					    (GRID:MAKE-FOREIGN-ARRAY
					     '(UNSIGNED-BYTE
					       8)
					     :DIMENSIONS
					     '3)))
				       (GRID:COPY-TO
					(SET-ALL V1
						 67)
					 'array '(unsigned-byte 8)))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL (LIST #(-64 -64 -64))
				    (MULTIPLE-VALUE-LIST
				     (LET ((V1
					    (GRID:MAKE-FOREIGN-ARRAY
					     '(SIGNED-BYTE
					       16)
					     :DIMENSIONS
					     '3)))
				       (GRID:COPY-TO
					(SET-ALL V1
						 -64)
					 'array '(signed-byte 16)))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL (LIST #(67 67 67))
				    (MULTIPLE-VALUE-LIST
				     (LET ((V1
					    (GRID:MAKE-FOREIGN-ARRAY
					     '(UNSIGNED-BYTE
					       16)
					     :DIMENSIONS
					     '3)))
				       (GRID:COPY-TO
					(SET-ALL V1
						 67)
					 'array '(unsigned-byte 16)))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL (LIST #(-64 -64 -64))
				    (MULTIPLE-VALUE-LIST
				     (LET ((V1
					    (GRID:MAKE-FOREIGN-ARRAY
					     '(SIGNED-BYTE
					       32)
					     :DIMENSIONS
					     '3)))
				       (GRID:COPY-TO
					(SET-ALL V1
						 -64)
					 'array '(signed-byte 32)))))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL (LIST #(67 67 67))
				    (MULTIPLE-VALUE-LIST
				     (LET ((V1
					    (GRID:MAKE-FOREIGN-ARRAY
					     '(UNSIGNED-BYTE
					       32)
					     :DIMENSIONS
					     '3)))
				       (GRID:COPY-TO
					(SET-ALL V1
						 67)
					 'array '(unsigned-byte 32)))))
  #+int64
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST #(-64 -64 -64))
   (MULTIPLE-VALUE-LIST
    (LET ((V1 (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 64) :DIMENSIONS '3)))
      (GRID:COPY-TO (SET-ALL V1 -64) 'array '(signed-byte 64)))))
  #+int64
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST #(67 67 67))
   (MULTIPLE-VALUE-LIST
    (LET ((V1 (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 64) :DIMENSIONS '3)))
      (GRID:COPY-TO (SET-ALL V1 67) 'array '(unsigned-byte 64))))))
