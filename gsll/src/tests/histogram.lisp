;; Regression test HISTOGRAM for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST HISTOGRAM
                       (LISP-UNIT:ASSERT-ERROR 'INPUT-DOMAIN
                                               (LET ((HISTO
                                                      (MAKE-HISTOGRAM 10)))
                                                 (SET-RANGES-UNIFORM HISTO
                                                                     0.0d0
                                                                     10.0d0)
                                                 (INCREMENT HISTO -2.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST 0.0d0)
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((HISTO
                                                                  (MAKE-HISTOGRAM
                                                                   10)))
                                                             (SET-RANGES-UNIFORM
                                                              HISTO 0.0d0
                                                              10.0d0)
                                                             (INCREMENT HISTO
                                                                        2.7d0)
                                                             (INCREMENT HISTO
                                                                        6.9d0
                                                                        2.0d0)
                                                             (GRID:GREF HISTO 1))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST 1.0d0)
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((HISTO
                                                                  (MAKE-HISTOGRAM
                                                                   10)))
                                                             (SET-RANGES-UNIFORM
                                                              HISTO 0.0d0
                                                              10.0d0)
                                                             (INCREMENT HISTO
                                                                        2.7d0)
                                                             (INCREMENT HISTO
                                                                        6.9d0
                                                                        2.0d0)
                                                             (GRID:GREF HISTO 2))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST 2.0d0)
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((HISTO
                                                                  (MAKE-HISTOGRAM
                                                                   10)))
                                                             (SET-RANGES-UNIFORM
                                                              HISTO 0.0d0
                                                              10.0d0)
                                                             (INCREMENT HISTO
                                                                        2.7d0)
                                                             (INCREMENT HISTO
                                                                        6.9d0
                                                                        2.0d0)
                                                             (GRID:GREF HISTO 6))))
                       (LISP-UNIT:ASSERT-ERROR 'INPUT-DOMAIN
                                               (LET ((HISTO
                                                      (MAKE-HISTOGRAM 10)))
                                                 (SET-RANGES-UNIFORM HISTO
                                                                     0.0d0
                                                                     10.0d0)
                                                 (INCREMENT HISTO 2.7d0)
                                                 (INCREMENT HISTO 6.9d0 2.0d0)
                                                 (GRID:GREF HISTO 16)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST 0.0d0 10.0d0)
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((HISTO
                                                                  (MAKE-HISTOGRAM
                                                                   10)))
                                                             (SET-RANGES-UNIFORM
                                                              HISTO 0.0d0
                                                              10.0d0)
                                                             (INCREMENT HISTO
                                                                        2.7d0)
                                                             (INCREMENT HISTO
                                                                        6.9d0
                                                                        2.0d0)
                                                             (VALUES
                                                              (MIN-RANGE HISTO)
                                                              (MAX-RANGE
                                                               HISTO)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST 10)
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((HISTO
                                                                  (MAKE-HISTOGRAM
                                                                   10)))
                                                             (SET-RANGES-UNIFORM
                                                              HISTO 0.0d0
                                                              10.0d0)
                                                             (INCREMENT HISTO
                                                                        2.7d0)
                                                             (INCREMENT HISTO
                                                                        6.9d0
                                                                        2.0d0)
                                                             (BINS HISTO))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST 5)
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((HISTO
                                                                  (MAKE-HISTOGRAM
                                                                   10)))
                                                             (SET-RANGES-UNIFORM
                                                              HISTO 0.0d0
                                                              10.0d0)
                                                             (INCREMENT HISTO
                                                                        2.7d0)
                                                             (INCREMENT HISTO
                                                                        6.9d0
                                                                        2.0d0)
                                                             (HISTOGRAM-FIND
                                                              HISTO 5.5d0)))))

