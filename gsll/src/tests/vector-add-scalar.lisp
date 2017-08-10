;; Regression test VECTOR-ADD-SCALAR for GSLL, automatically generated
;;
;; Copyright 2009, 2014 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST VECTOR-ADD-SCALAR
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(-16.31f0 26.43f0 21.48f0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((V1
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
                                             '(-34.5f0 8.24f0 3.29f0))))
                           (GRID:COPY-TO (ELT+ V1 18.19d0) 'array 'single-float))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(-16.31d0 26.43d0 21.48d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((V1
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '(-34.5d0 8.24d0 3.29d0))))
                           (GRID:COPY-TO (ELT+ V1 18.19d0) 'array 'double-float))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-45 -49 89))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     8)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71))))
                                                             (GRID:COPY-TO
                                                              (ELT+ V1
                                                                   18.19d0)
							       'array '(signed-byte 8)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(85 62 207))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(UNSIGNED-BYTE
                                                                     8)
                                                                   :INITIAL-CONTENTS
                                                                   '(67 44
                                                                     189))))
                                                             (GRID:COPY-TO
                                                              (ELT+ V1
                                                                   18.19d0)
							       'array '(unsigned-byte 8)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-45 -49 89))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     16)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71))))
                                                             (GRID:COPY-TO
                                                              (ELT+ V1
                                                                   18.19d0)
							       'array '(signed-byte 16)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(85 62 207))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(UNSIGNED-BYTE
                                                                     16)
                                                                   :INITIAL-CONTENTS
                                                                   '(67 44
                                                                     189))))
                                                             (GRID:COPY-TO
                                                              (ELT+ V1
                                                                   18.19d0)
							       'array '(unsigned-byte 16)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-45 -49 89))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     32)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71))))
                                                             (GRID:COPY-TO
                                                              (ELT+ V1
                                                                   18.19d0)
							       'array '(signed-byte 32)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(85 62 207))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(UNSIGNED-BYTE
                                                                     32)
                                                                   :INITIAL-CONTENTS
                                                                   '(67 44
                                                                     189))))
                                                             (GRID:COPY-TO
                                                              (ELT+ V1
                                                                   18.19d0)
							       'array '(unsigned-byte 32)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-45 -49 89))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     64)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71))))
                                                             (GRID:COPY-TO
                                                              (ELT+ V1
                                                                   18.19d0)
							       'array '(signed-byte 64)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(85 62 207))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(UNSIGNED-BYTE
                                                                     64)
                                                                   :INITIAL-CONTENTS
                                                                   '(67 44
                                                                     189))))
                                                             (GRID:COPY-TO
                                                              (ELT+ V1
                                                                   18.19d0)
							       'array '(unsigned-byte 64))))))

