;; Regression test MATRIX-PRODUCT-SYMMETRIC for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MATRIX-PRODUCT-SYMMETRIC
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((74519.41 -16747.695 61311.69)
                             (-3264.9219 6201.1797 -56698.92)
                             (57711.813 52563.74 65404.063)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
                                             '((-34.5 8.24 3.29)
                                               (-8.93 34.12 -6.15)
                                               (49.27 -13.49 32.5))))
                               (M2
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
                                             '((42.73 -17.24 43.31)
                                               (-16.12 -8.25 21.44)
                                               (-49.08 -39.66 -49.46))))
                               (M3
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
                                             '((19.68 -5.55 -8.82)
                                               (25.37 -30.58 31.67)
                                               (29.36 -33.24 -27.03))))
                               (S1 -41.67)
                               (S2 42.0))
                           (GRID:COPY-TO
                            (MATRIX-PRODUCT-SYMMETRIC M1 M2 M3 S1 S2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((74519.41329d0 -16747.696062d0
                              61311.694176000005d0)
                             (-3264.923076000001d0 6201.180462000002d0
                              -56698.92695400001d0)
                             (57711.81710100001d0 52563.740607d0
                              65404.066887d0)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '((-34.5d0 8.24d0 3.29d0)
                                               (-8.93d0 34.12d0 -6.15d0)
                                               (49.27d0 -13.49d0 32.5d0))))
                               (M2
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '((42.73d0 -17.24d0 43.31d0)
                                               (-16.12d0 -8.25d0 21.44d0)
                                               (-49.08d0 -39.66d0 -49.46d0))))
                               (M3
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '((19.68d0 -5.55d0 -8.82d0)
                                               (25.37d0 -30.58d0 31.67d0)
                                               (29.36d0 -33.24d0 -27.03d0))))
                               (S1 -41.67d0)
                               (S2 42.0d0))
                           (GRID:COPY-TO
                            (MATRIX-PRODUCT-SYMMETRIC M1 M2 M3 S1 S2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(72971.1 24989.41 -82037.61))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
                                             '((-34.5 8.24 3.29)
                                               (-8.93 34.12 -6.15)
                                               (49.27 -13.49 32.5))))
                               (V1
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
                                             '(42.73 -17.24 43.31)))
                               (V3
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
                                             '(-16.12 -8.25 21.44)))
                               (S1 -49.08)
                               (S2 -39.66))
                           (GRID:COPY-TO
                            (MATRIX-PRODUCT-SYMMETRIC M1 V1 V3 S1 S2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #(72971.10171599999d0 24989.409107999996d0
                           -82037.597316d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '((-34.5d0 8.24d0 3.29d0)
                                               (-8.93d0 34.12d0 -6.15d0)
                                               (49.27d0 -13.49d0 32.5d0))))
                               (V1
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '(42.73d0 -17.24d0 43.31d0)))
                               (V3
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '(-16.12d0 -8.25d0 21.44d0)))
                               (S1 -49.08d0)
                               (S2 -39.66d0))
                           (GRID:COPY-TO
                            (MATRIX-PRODUCT-SYMMETRIC M1 V1 V3 S1 S2))))))

