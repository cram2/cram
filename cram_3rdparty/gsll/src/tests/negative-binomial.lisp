;; Regression test NEGATIVE-BINOMIAL for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST NEGATIVE-BINOMIAL
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST (LIST 15 21 19 15 8 18 23 18 33 16 10))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 10 COLLECT
	   (sample rng :negative-binomial :probability 0.4d0 :n 12.0d0)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.0056984767089869d0)
   (MULTIPLE-VALUE-LIST
    (NEGATIVE-BINOMIAL-PDF 5 0.4d0 12.0d0)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.010594202555514881d0)
   (MULTIPLE-VALUE-LIST
    (NEGATIVE-BINOMIAL-P 5 0.4d0 12.0d0)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.9894057974444851d0)
   (MULTIPLE-VALUE-LIST
    (NEGATIVE-BINOMIAL-Q 5 0.4d0 12.0d0)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST (LIST 15 21 19 15 8 18 23 18 33 16 10))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 10 COLLECT
	   (sample rng :pascal :probability 0.4d0 :n 12)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.0056984767089869d0)
   (MULTIPLE-VALUE-LIST (PASCAL-PDF 5 0.4d0 12)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.010594202555514881d0)
   (MULTIPLE-VALUE-LIST (PASCAL-P 5 0.4d0 12)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.9894057974444851d0)
   (MULTIPLE-VALUE-LIST (PASCAL-Q 5 0.4d0 12))))

