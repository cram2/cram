;; Regression test MATRIX-SET-ALL for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST MATRIX-SET-ALL
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((-34.5 -34.5 -34.5)
                             (-34.5 -34.5 -34.5)
                             (-34.5 -34.5 -34.5)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :DIMENSIONS '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 -34.5)))))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((-34.5d0 -34.5d0 -34.5d0)
                             (-34.5d0 -34.5d0 -34.5d0)
                             (-34.5d0 -34.5d0 -34.5d0)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 -34.5d0)))))
		       #+fsbv
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((#C(-34.5 8.24) #C(-34.5 8.24) #C(-34.5 8.24))
                             (#C(-34.5 8.24) #C(-34.5 8.24) #C(-34.5 8.24))
                             (#C(-34.5 8.24) #C(-34.5 8.24) #C(-34.5 8.24))))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(COMPLEX SINGLE-FLOAT)
                                             :DIMENSIONS '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 #C(-34.5 8.24))))))
		       #+fsbv
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #2A((#C(-34.5d0 8.24d0) #C(-34.5d0 8.24d0)
                              #C(-34.5d0 8.24d0))
                             (#C(-34.5d0 8.24d0) #C(-34.5d0 8.24d0)
                              #C(-34.5d0 8.24d0))
                             (#C(-34.5d0 8.24d0) #C(-34.5d0 8.24d0)
                              #C(-34.5d0 8.24d0))))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(COMPLEX DOUBLE-FLOAT)
                                             :DIMENSIONS '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 #C(-34.5d0 8.24d0))))))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST #2A((-64 -64 -64) (-64 -64 -64) (-64 -64 -64)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 8) :DIMENSIONS
                                             '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 -64)))))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST #2A((67 67 67) (67 67 67) (67 67 67)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 8) :DIMENSIONS
                                             '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 67)))))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST #2A((-64 -64 -64) (-64 -64 -64) (-64 -64 -64)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 16) :DIMENSIONS
                                             '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 -64)))))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST #2A((67 67 67) (67 67 67) (67 67 67)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 16) :DIMENSIONS
                                             '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 67)))))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST #2A((-64 -64 -64) (-64 -64 -64) (-64 -64 -64)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 32) :DIMENSIONS
                                             '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 -64)))))
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST #2A((67 67 67) (67 67 67) (67 67 67)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 32) :DIMENSIONS
                                             '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 67)))))
		       #+int64
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST #2A((-64 -64 -64) (-64 -64 -64) (-64 -64 -64)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(SIGNED-BYTE 64) :DIMENSIONS
                                             '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 -64)))))
		       #+int64
                       (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
                        (LIST #2A((67 67 67) (67 67 67) (67 67 67)))
                        (MULTIPLE-VALUE-LIST
                         (LET ((M1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 64) :DIMENSIONS
                                             '(3 3))))
                           (GRID:COPY-TO (SET-ALL M1 67))))))

