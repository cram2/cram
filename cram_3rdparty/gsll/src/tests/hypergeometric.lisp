;; Regression test HYPERGEOMETRIC for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST HYPERGEOMETRIC
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 3.7621956910836287d0 9.207469176703522d-14)
                        (MULTIPLE-VALUE-LIST (HYPERGEOMETRIC-0F1 0.5d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 5.43656365691809d0 6.0357981467508045d-15)
                        (MULTIPLE-VALUE-LIST (HYPERGEOMETRIC-1F1 2 1 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 5.43656365691809d0 6.0357981467508045d-15)
                        (MULTIPLE-VALUE-LIST
                         (HYPERGEOMETRIC-1F1 2.0d0 1.0d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.19269472464638984d0 2.5468520714552053d-12)
                        (MULTIPLE-VALUE-LIST
                         (HYPERGEOMETRIC-U 2.0d0 1.0d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.19269472464638984d0 2.5468520714552053d-12)
                        (MULTIPLE-VALUE-LIST (HYPERGEOMETRIC-U 2 1 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.19269472464638984d0 0 2.5468520714552053d-12)
                        (MULTIPLE-VALUE-LIST
                         (HYPERGEOMETRIC-U-E10 2.0d0 1.0d0 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.19269472464638984d0 0 2.5468520714552053d-12)
                        (MULTIPLE-VALUE-LIST (HYPERGEOMETRIC-U-E10 2 1 1.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 2.29739670999407d0 2.0404981793068d-15)
                        (MULTIPLE-VALUE-LIST
                         (HYPERGEOMETRIC-2F1 1.0d0 1.2d0 1.0d0 0.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 6.629594934547447d0 5.761143589129193d-14)
                        (MULTIPLE-VALUE-LIST
                         (HYPERGEOMETRIC-2F1-CONJ #C(1.0d0 0.5d0) 0.5d0
                                                  0.6d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 3.7403484052126372d0 5.884558632199498d-14)
                        (MULTIPLE-VALUE-LIST
                         (HYPERGEOMETRIC-2F1-CONJ-RENORM #C(1.0d0 0.5d0) 0.5d0
                                                         0.6d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 2.29739670999407d0 3.0607472689602005d-15)
                        (MULTIPLE-VALUE-LIST
                         (HYPERGEOMETRIC-2F1-RENORM 1.0d0 1.2d0 1.0d0 0.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.043513924125598485d0 3.81571654717325d-15)
                        (MULTIPLE-VALUE-LIST
                         (HYPERGEOMETRIC-2F0 1.0d0 2.0d0 -20.0d0))))

