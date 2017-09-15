;; Regression test DEBYE for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST DEBYE
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.7775046341122482d0 4.117962028082377d-16)
                        (MULTIPLE-VALUE-LIST (DEBYE-1 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.7078784756278294d0 4.948781517277596d-16)
                        (MULTIPLE-VALUE-LIST (DEBYE-2 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.6744155640778147d0 5.450931753448871d-16)
                        (MULTIPLE-VALUE-LIST (DEBYE-3 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.654874068886737d0 5.769372522202218d-16)
                        (MULTIPLE-VALUE-LIST (DEBYE-4 1.0d0))))

