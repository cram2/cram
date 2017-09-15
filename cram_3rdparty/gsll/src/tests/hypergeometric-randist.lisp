;; Regression test HYPERGEOMETRIC-RANDIST for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST HYPERGEOMETRIC-RANDIST
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST (LIST 2 1 0 0 1 1 3 1 0 1 3))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 10 COLLECT
	   (sample rng :hypergeometric :n1 3 :n2 6 :tt 3)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.35714285714285693d0)
   (MULTIPLE-VALUE-LIST (HYPERGEOMETRIC-PDF 0 2 6 3)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.892857142857143d0)
   (MULTIPLE-VALUE-LIST (HYPERGEOMETRIC-P 1 2 6 3)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.10714285714285704d0)
   (MULTIPLE-VALUE-LIST (HYPERGEOMETRIC-Q 1 2 6 3))))

