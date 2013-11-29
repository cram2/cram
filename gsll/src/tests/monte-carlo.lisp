;; Regression test MONTE-CARLO for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MONTE-CARLO
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.4122087033540667d0 0.013435861456267064d0)
                        (MULTIPLE-VALUE-LIST (RANDOM-WALK-PLAIN-EXAMPLE)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.3895297058825096d0 0.0050106269732269415d0)
                        (MULTIPLE-VALUE-LIST (RANDOM-WALK-MISER-EXAMPLE)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.3931632739551914d0 1.4981164744582692d-4)
                        (MULTIPLE-VALUE-LIST (RANDOM-WALK-VEGAS-EXAMPLE))))

