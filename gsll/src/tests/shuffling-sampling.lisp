;; Regression test SHUFFLING-SAMPLING for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST SHUFFLING-SAMPLING
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(4 3 6 1 5 7 2 8))
   (MULTIPLE-VALUE-LIST
    (let ((rng (make-random-number-generator +mt19937+ 0))
	  (v1 #31m(1 2 3 4 5 6 7 8)))
      (grid:copy-to (sample rng :shuffle :base v1)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(2 3 5 8))
   (MULTIPLE-VALUE-LIST
    (let ((rng (make-random-number-generator +mt19937+ 0))
	  (v1 #31m(1 2 3 4 5 6 7 8)))
      (grid:copy-to (sample rng :choose-random :src v1 :dest 4)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST #(8 2 3 8 2 4 8 6 5 6))
   (MULTIPLE-VALUE-LIST
    (let ((rng (make-random-number-generator +mt19937+ 0))
	  (v1 #31m(1 2 3 4 5 6 7 8)))
      (grid:copy-to (sample rng :random-sample :src v1 :dest 10))))))
