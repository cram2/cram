;; Regression test COMBINATION for GSLL, automatically generated
;;
;; Copyright 2009 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST COMBINATION
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 4)
   (MULTIPLE-VALUE-LIST
    (LET ((COMB (MAKE-COMBINATION 4 2)))
      (COMBINATION-RANGE COMB))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 2)
   (MULTIPLE-VALUE-LIST
    (LET ((COMB (MAKE-COMBINATION 4 2)))
      (SIZE COMB))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST (LIST #(0 1) #(0 2) #(0 3) #(1 2) #(1 3) #(2 3)))
   (MULTIPLE-VALUE-LIST
    (LET ((COMB (MAKE-COMBINATION 4 2)))
      (INIT-FIRST COMB)
      (LOOP COLLECT (COPY-SEQ (GRID:COPY-TO COMB)) WHILE
	   (COMBINATION-NEXT COMB)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST (LIST #(2 3) #(1 3) #(1 2) #(0 3) #(0 2) #(0 1)))
   (MULTIPLE-VALUE-LIST
    (LET ((COMB (MAKE-COMBINATION 4 2)))
      (INIT-LAST COMB)
      (LOOP COLLECT (COPY-SEQ (GRID:COPY-TO COMB)) WHILE
	   (COMBINATION-PREVIOUS COMB)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST #() #(0) #(1) #(2) #(3) #(0 1) #(0 2) #(0 3)
	  #(1 2) #(1 3) #(2 3) #(0 1 2) #(0 1 3) #(0 2 3)
	  #(1 2 3) #(0 1 2 3)))
   (MULTIPLE-VALUE-LIST
    (LOOP FOR I FROM 0 TO 4 APPEND
	 (LET ((COMB (MAKE-COMBINATION 4 I)))
	   (INIT-FIRST COMB)
	   (LOOP COLLECT (grid:contents comb) WHILE
		(COMBINATION-NEXT COMB)))))))
