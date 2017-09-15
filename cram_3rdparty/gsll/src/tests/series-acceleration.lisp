;; Regression test SERIES-ACCELERATION for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST SERIES-ACCELERATION
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.5961632439130233d0 20 1.5759958390005426d0 13
                              1.6449340669228176d0 8.883604962761638d-11
                              7.459122208786084d-11)
                        (MULTIPLE-VALUE-LIST (ACCELERATION-EXAMPLE NIL))))

