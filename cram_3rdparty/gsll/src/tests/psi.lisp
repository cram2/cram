;; Regression test PSI for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST PSI
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.2561176684318005d0 2.789141514262906d-16)
                        (MULTIPLE-VALUE-LIST (PSI 4)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.2561176684318005d0 2.846229783626858d-16)
                        (MULTIPLE-VALUE-LIST (PSI 4.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.7145915153739806d0 1.925418559790263d-14)
                        (MULTIPLE-VALUE-LIST (PSI-1+IY 2.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.2838229557371153d0 6.302135607530242d-17)
                        (MULTIPLE-VALUE-LIST (PSI-1 4)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.28382295573711525d0 1.764597970108467d-15)
                        (MULTIPLE-VALUE-LIST (PSI-1 4.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.0800397322451145d0 5.222647053360405d-16)
                        (MULTIPLE-VALUE-LIST (PSI-N 2 4.0d0))))

