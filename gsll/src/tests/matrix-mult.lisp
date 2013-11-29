;; Regression test MATRIX-MULT for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MATRIX-MULT
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((-1474.1849 -142.05759 142.4899)
                             (143.95161 -281.49 -131.856)
                             (-2418.1716 535.01337 -1607.45)))
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
                                               (-49.08 -39.66 -49.46)))))
                           (GRID:COPY-TO (ELT* M1 M2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((-1474.185d0 -142.05759999999998d0 142.4899d0)
                             (143.9516d0 -281.48999999999995d0
                              -131.85600000000002d0)
                             (-2418.1716d0 535.0133999999999d0 -1607.45d0)))
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
                                               (-49.08d0 -39.66d0 -49.46d0)))))
                           (GRID:COPY-TO (ELT* M1 M2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #2A((0 -112 82) (-39 44 80) (-71 110 -84)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS
                                             '((-64 -68 71) (-91 52 -10)
                                               (73 -5 123))))
                               (M2
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :INITIAL-CONTENTS
                                             '((32 28 30) (37 -73 -8)
                                               (-15 -22 68)))))
                           (GRID:COPY-TO (ELT* M1 M2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #2A((84 184 7) (176 86 172) (164 139 252)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 8)
                                             :INITIAL-CONTENTS
                                             '((67 44 189) (116 163 140)
                                               (161 215 98))))
                               (M2
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 8)
                                             :INITIAL-CONTENTS
                                             '((28 10 19) (28 178 217)
                                               (36 109 222)))))
                           (GRID:COPY-TO (ELT* M1 M2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((-2048 -1904 2130)
                             (-3367 -3796 80)
                             (-1095 110 8364)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 16)
                                             :INITIAL-CONTENTS
                                             '((-64 -68 71) (-91 52 -10)
                                               (73 -5 123))))
                               (M2
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 16)
                                             :INITIAL-CONTENTS
                                             '((32 28 30) (37 -73 -8)
                                               (-15 -22 68)))))
                           (GRID:COPY-TO (ELT* M1 M2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((1876 440 3591)
                             (3248 29014 30380)
                             (5796 23435 21756)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 16)
                                             :INITIAL-CONTENTS
                                             '((67 44 189) (116 163 140)
                                               (161 215 98))))
                               (M2
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 16)
                                             :INITIAL-CONTENTS
                                             '((28 10 19) (28 178 217)
                                               (36 109 222)))))
                           (GRID:COPY-TO (ELT* M1 M2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((-2048 -1904 2130)
                             (-3367 -3796 80)
                             (-1095 110 8364)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 32)
                                             :INITIAL-CONTENTS
                                             '((-64 -68 71) (-91 52 -10)
                                               (73 -5 123))))
                               (M2
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 32)
                                             :INITIAL-CONTENTS
                                             '((32 28 30) (37 -73 -8)
                                               (-15 -22 68)))))
                           (GRID:COPY-TO (ELT* M1 M2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((1876 440 3591)
                             (3248 29014 30380)
                             (5796 23435 21756)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 32)
                                             :INITIAL-CONTENTS
                                             '((67 44 189) (116 163 140)
                                               (161 215 98))))
                               (M2
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 32)
                                             :INITIAL-CONTENTS
                                             '((28 10 19) (28 178 217)
                                               (36 109 222)))))
                           (GRID:COPY-TO (ELT* M1 M2)))))
		       #+int64
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((-2048 -1904 2130)
                             (-3367 -3796 80)
                             (-1095 110 8364)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 64)
                                             :INITIAL-CONTENTS
                                             '((-64 -68 71) (-91 52 -10)
                                               (73 -5 123))))
                               (M2
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 64)
                                             :INITIAL-CONTENTS
                                             '((32 28 30) (37 -73 -8)
                                               (-15 -22 68)))))
                           (GRID:COPY-TO (ELT* M1 M2)))))
		       #+int64
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((1876 440 3591)
                             (3248 29014 30380)
                             (5796 23435 21756)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 64)
                                             :INITIAL-CONTENTS
                                             '((67 44 189) (116 163 140)
                                               (161 215 98))))
                               (M2
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 64)
                                             :INITIAL-CONTENTS
                                             '((28 10 19) (28 178 217)
                                               (36 109 222)))))
                           (GRID:COPY-TO (ELT* M1 M2))))))

