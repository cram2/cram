;; Regression test DIRICHLET for GSLL, automatically generated
;;
;; Copyright 2009, 2014 Liam M. Healy
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
	       (alpha
		(GRID:MAKE-FOREIGN-ARRAY
		 'DOUBLE-FLOAT :INITIAL-CONTENTS
		 '(1.0d0 2.0d0 3.0d0 4.0d0))))
	   (grid:copy-to (sample rng :dirichlet :alpha alpha) 'array 'double-float))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (list 3.483648000000004d7)
   (MULTIPLE-VALUE-LIST
       (LET ((ALPHA
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(1.0d0 2.0d0 3.0d0 4.0d0)))
	     (THETA
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(1.0d0 2.0d0 3.0d0 4.0d0))))
	 (DIRICHLET-PDF ALPHA THETA))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (list 17.366175671549307d0)
   (MULTIPLE-VALUE-LIST
       (LET ((ALPHA
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(1.0d0 2.0d0 3.0d0 4.0d0)))
	     (THETA
	      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
				       '(1.0d0 2.0d0 3.0d0 4.0d0))))
	 (DIRICHLET-LOG-PDF ALPHA THETA)))))
