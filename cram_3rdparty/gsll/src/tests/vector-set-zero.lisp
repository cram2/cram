;; Regression test VECTOR-SET-ZERO for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST VECTOR-SET-ZERO
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0.0f0 0.0f0 0.0f0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS '(-34.5f0 8.24f0 3.29f0))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array 'single-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0.0d0 0.0d0 0.0d0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS '(-34.5d0 8.24d0 3.29d0))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array 'double-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(#C(0.0f0 0.0f0) #C(0.0f0 0.0f0) #C(0.0f0 0.0f0)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX SINGLE-FLOAT)
	    :INITIAL-CONTENTS '(#C(-34.5f0 8.24f0) #C(3.29f0 -8.93f0) #C(34.12f0 -6.15f0)))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array '(complex single-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(#C(0.0d0 0.0d0) #C(0.0d0 0.0d0) #C(0.0d0 0.0d0)))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(COMPLEX DOUBLE-FLOAT)
	    :INITIAL-CONTENTS
	    '(#C(-34.5d0 8.24d0) #C(3.29d0 -8.93d0) #C(34.12d0 -6.15d0)))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array '(complex double-float)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 0 0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1 (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS '(-64 -68 71))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array '(signed-byte 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 0 0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY
	    '(UNSIGNED-BYTE 8)
	    :INITIAL-CONTENTS '(67 44 189))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array '(unsigned-byte 8)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 0 0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1 (GRID:MAKE-FOREIGN-ARRAY
	       '(SIGNED-BYTE 16) :INITIAL-CONTENTS '(-64 -68 71))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array '(signed-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 0 0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 16) :INITIAL-CONTENTS '(67 44 189))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array '(unsigned-byte 16)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 0 0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1 (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 32) :INITIAL-CONTENTS '(-64 -68 71))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array '(signed-byte 32)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 0 0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1 (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 32) :INITIAL-CONTENTS '(67 44 189))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array '(unsigned-byte 32)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 0 0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1 (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 64) :INITIAL-CONTENTS '(-64 -68 71))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array '(signed-byte 64)))))
  #+int64
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 0 0))
   (MULTIPLE-VALUE-LIST
    (LET ((V1
	   (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 64) :INITIAL-CONTENTS '(67 44 189))))
      (SET-ZERO V1)
      (GRID:COPY-TO V1 'array '(unsigned-byte 64))))))

