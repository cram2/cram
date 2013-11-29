;; Regression test PERMUTATION for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST PERMUTATION
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 2)
   (MULTIPLE-VALUE-LIST
    (LET ((PERM-1 (MAKE-PERMUTATION 4 T)))
      (grid:gref PERM-1 2))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 1 2 3))
   (MULTIPLE-VALUE-LIST
    (LET ((PERM-1 (MAKE-PERMUTATION 4 T)))
      (GRID:COPY-TO PERM-1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(3 2 1 0))
   (MULTIPLE-VALUE-LIST
    (LET ((PERM-1 (MAKE-PERMUTATION 4 T)))
      (SET-IDENTITY PERM-1)
      (GRID:COPY-TO (PERMUTATION-REVERSE PERM-1)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 3 1 2))
   (MULTIPLE-VALUE-LIST
    (let			;permutation-next, permutation-inverse
	((perm-1 (make-permutation 4 t)))
      (set-identity perm-1)
      (permutation-next perm-1)
      (permutation-next perm-1)
      (permutation-next perm-1)
      (grid:copy-to (permutation-inverse perm-1)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(0 3 2 1))
   (MULTIPLE-VALUE-LIST
    (LET ((PERM-1 (MAKE-PERMUTATION 4 T)))
      (SET-IDENTITY PERM-1)
      (SWAP-ELEMENTS PERM-1 1 3)
      (GRID:COPY-TO PERM-1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(33 44 11 22))
   (MULTIPLE-VALUE-LIST
    (LET ((PERM-1 (MAKE-PERMUTATION 4 T))
	  (INTVEC (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 32)
					   :INITIAL-CONTENTS '(11 22 33 44))))
      (SET-IDENTITY PERM-1)
      (SWAP-ELEMENTS PERM-1 1 3)
      (SWAP-ELEMENTS PERM-1 0 2)
      (PERMUTE PERM-1 INTVEC)
      (GRID:COPY-TO INTVEC))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 3)
   (MULTIPLE-VALUE-LIST
    (LET ((PERM-1 (MAKE-PERMUTATION 4 T)))
      (SET-IDENTITY PERM-1)
      (SWAP-ELEMENTS PERM-1 1 3)
      (INVERSIONS PERM-1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 3)
   (MULTIPLE-VALUE-LIST
    (LET ((PERM-1 (MAKE-PERMUTATION 4 T)))
      (SET-IDENTITY PERM-1)
      (SWAP-ELEMENTS PERM-1 1 3)
      (LINEAR-CYCLES PERM-1))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 2)
   (MULTIPLE-VALUE-LIST
    (LET ((PERM-1 (MAKE-PERMUTATION 4 T)))
      (SET-IDENTITY PERM-1)
      (SWAP-ELEMENTS PERM-1 1 3)
      (SWAP-ELEMENTS PERM-1 0 2)
      (CANONICAL-CYCLES PERM-1)))))

