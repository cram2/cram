;; Regression test TRANSPORT for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST TRANSPORT
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 2.806666404563118d0 2.2867923780257255d-15)
                        (MULTIPLE-VALUE-LIST (TRANSPORT-2 4.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 4.579217437229157d0 3.242324689309112d-15)
                        (MULTIPLE-VALUE-LIST (TRANSPORT-3 4.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 10.731932392998623d0 1.0925209116254758d-14)
                        (MULTIPLE-VALUE-LIST (TRANSPORT-4 4.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 29.488339015245842d0 3.204450601879883d-14)
                        (MULTIPLE-VALUE-LIST (TRANSPORT-5 4.0d0))))

