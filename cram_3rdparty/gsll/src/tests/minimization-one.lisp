;; Regression test MINIMIZATION-ONE for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MINIMIZATION-ONE
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST 6 3.141474321994987d0 3.1415930343642042d0
	 3.141592654724622d0 1.134828675475319d-9
	 1.1871236921701112d-4)
   (MULTIPLE-VALUE-LIST
    (MINIMIZATION-ONE-EXAMPLE +BRENT-FMINIMIZER+ NIL
			      NIL)))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST 24 3.1413247152275243d0 3.1419619412229034d0
	 3.1415652995716155d0 -2.7354018177661032d-5
	 6.37225995379076d-4)
   (MULTIPLE-VALUE-LIST
    (MINIMIZATION-ONE-EXAMPLE +GOLDEN-SECTION-FMINIMIZER+
			      NIL NIL)))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST 6 3.141474321994987d0 3.1415930343642042d0
	 3.141592654724622d0 1.134828675475319d-9
	 1.1871236921701112d-4)
   (MULTIPLE-VALUE-LIST
    (MINIMIZATION-ONE-EXAMPLE +BRENT-FMINIMIZER+ NIL T)))
  (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
   (LIST 24 3.1413247152275243d0 3.1419619412229034d0
	 3.1415652995716155d0 -2.7354018177661032d-5
	 6.37225995379076d-4)
   (MULTIPLE-VALUE-LIST
    (MINIMIZATION-ONE-EXAMPLE +GOLDEN-SECTION-FMINIMIZER+
			      NIL T))))

