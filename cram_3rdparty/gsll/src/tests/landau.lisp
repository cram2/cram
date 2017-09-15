;; Regression test LANDAU for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST LANDAU
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST 3880.037426254597d0 -0.6953200314545297d0
	  -0.02354364646600932d0 21.329209630030316d0
	  -0.3062224704714883d0 1.2424186669362394d0
	  26.146168479649152d0 4.337217640968217d0
	  1.6799546281085946d0 4.2475719218268395d0
	  4.681506208977819d0))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 10 COLLECT (sample rng :landau)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.17331968995860203d0)
   (MULTIPLE-VALUE-LIST (LANDAU-PDF 0.25d0))))

