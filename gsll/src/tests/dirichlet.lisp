;; Regression test DIRICHLET for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST DIRICHLET
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    #(0.12448073544131681d0 0.19182353706734917d0
      0.46054388544826397d0 0.22315184204307006d0))
   (MULTIPLE-VALUE-LIST
    (let ((rng (make-random-number-generator +mt19937+ 0))
	  (alpha #m(1.0d0 2.0d0 3.0d0 4.0d0)))
      (grid:copy-to (sample rng :dirichlet :alpha alpha)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 1080.0000000000025d0)
   (MULTIPLE-VALUE-LIST
    (LET ((ALPHA
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
			'(1.0d0 2.0d0 3.0d0 4.0d0)))
	  (THETA
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
			'(1.0d0 2.0d0 3.0d0 4.0d0))))
      (DIRICHLET-PDF ALPHA THETA))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 6.984716320118268d0)
   (MULTIPLE-VALUE-LIST
    (LET ((ALPHA
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
			'(1.0d0 2.0d0 3.0d0 4.0d0)))
	  (THETA
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
			'(1.0d0 2.0d0 3.0d0 4.0d0))))
      (DIRICHLET-LOG-PDF ALPHA THETA)))))

