;; Regression test HIGHER-MOMENTS for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST HIGHER-MOMENTS
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         (LIST 0.2765118983985497d0 0.2765118983985497d0
                               -2.333333333333333d0 -2.333333333333333d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((VEC
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '(-3.21d0 1.0d0 12.8d0))))
                           (LET* ((MEAN (MEAN VEC))
                                  (SD (STANDARD-DEVIATION VEC MEAN)))
                             (LIST (SKEWNESS VEC) (SKEWNESS VEC MEAN SD)
                                   (KURTOSIS VEC) (KURTOSIS VEC MEAN SD)))))))

