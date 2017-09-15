;; Regression test SYNCHROTRON for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST SYNCHROTRON
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.052827396697912476d0 4.825849878208132d-14)
                        (MULTIPLE-VALUE-LIST (SYNCHROTRON-1 4.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.04692320582614684d0 6.854449168174663d-14)
                        (MULTIPLE-VALUE-LIST (SYNCHROTRON-2 4.0d0))))

