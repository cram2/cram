;; Regression test BINOMIAL for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST BINOMIAL
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST (LIST 11 3 4 8 4 5 8 6 5 6 6))
                        (MULTIPLE-VALUE-LIST
                         (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
                           (LOOP FOR I FROM 0 TO 10 COLLECT
                                 (BINOMIAL RNG 0.4d0 12)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.22703033548799986d0)
                        (MULTIPLE-VALUE-LIST (BINOMIAL-PDF 5 0.4d0 12)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.6652085575680018d0)
                        (MULTIPLE-VALUE-LIST (BINOMIAL-P 5 0.4d0 12)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.33479144243199815d0)
                        (MULTIPLE-VALUE-LIST (BINOMIAL-Q 5 0.4d0 12))))

