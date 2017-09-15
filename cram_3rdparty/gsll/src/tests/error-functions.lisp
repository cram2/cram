;; Regression test ERROR-FUNCTIONS for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST ERROR-FUNCTIONS
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.8427007929497149d0 7.789237746491556d-16)
                        (MULTIPLE-VALUE-LIST (ERF 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.1572992070502851d0 4.0468944536809554d-16)
                        (MULTIPLE-VALUE-LIST (ERFC 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -1.8496055099332485d0 3.394126565390616d-15)
                        (MULTIPLE-VALUE-LIST (LOG-ERFC 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.24197072451914334d0 1.611848817878303d-16)
                        (MULTIPLE-VALUE-LIST (ERF-Z 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.15865525393145707d0 2.832400331480832d-16)
                        (MULTIPLE-VALUE-LIST (ERF-Q 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.5251352761609807d0 5.532094155354489d-15)
                        (MULTIPLE-VALUE-LIST (HAZARD 1.0d0))))

