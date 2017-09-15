;; Regression test ZETA for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST ZETA
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.6449340668482264d0 7.304974700020789d-16)
                        (MULTIPLE-VALUE-LIST (ZETA 2)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 2.612375348685493d0 1.419471177334903d-14)
                        (MULTIPLE-VALUE-LIST (ZETA 1.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.6449340668482264d0 2.8640826015201633d-16)
                        (MULTIPLE-VALUE-LIST (ZETA-1 2)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.612375348685493d0 1.419471177334903d-14)
                        (MULTIPLE-VALUE-LIST (ZETA-1 1.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.4037797688568256d0 8.104244828616706d-15)
                        (MULTIPLE-VALUE-LIST (HURWITZ-ZETA 1.5d0 2.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.7651470246254092d0 1.0661276646941275d-14)
                        (MULTIPLE-VALUE-LIST (ETA 1.5d0))))

