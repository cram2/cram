;; Regression test VECTOR-MULT-SCALAR for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST VECTOR-MULT-SCALAR
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(-47.955 11.4536 4.5731))
                        (MULTIPLE-VALUE-LIST
                         (LET ((V1
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
                                             '(-34.5 8.24 3.29))))
                           (GRID:COPY-TO (ELT* V1 1.39d0) 'array 'single-float))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(-47.955d0 11.4536d0 4.5731d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((V1
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '(-34.5d0 8.24d0 3.29d0))))
                           (GRID:COPY-TO (ELT* V1 1.39d0) 'array 'double-float))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-88 -94 98))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     8)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71))))
                                                             (GRID:COPY-TO
                                                              (ELT* V1
                                                                   1.39d0)
							       'array '(SIGNED-BYTE 8)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(93 61 6))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(UNSIGNED-BYTE
                                                                     8)
                                                                   :INITIAL-CONTENTS
                                                                   '(67 44
                                                                     189))))
                                                             (GRID:COPY-TO
                                                              (ELT* V1
                                                                   1.39d0)
							       'array '(unSIGNED-BYTE 8)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-88 -94 98))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     16)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71))))
                                                             (GRID:COPY-TO
                                                              (ELT* V1
                                                                   1.39d0)
							       'array '(SIGNED-BYTE 16)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(93 61 262))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(UNSIGNED-BYTE
                                                                     16)
                                                                   :INITIAL-CONTENTS
                                                                   '(67 44
                                                                     189))))
                                                             (GRID:COPY-TO
                                                              (ELT* V1
                                                                   1.39d0)
							       'array '(unSIGNED-BYTE 16)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-88 -94 98))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     32)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71))))
                                                             (GRID:COPY-TO
                                                              (ELT* V1
                                                                   1.39d0)
							       'array '(SIGNED-BYTE 32)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(93 61 262))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(UNSIGNED-BYTE
                                                                     32)
                                                                   :INITIAL-CONTENTS
                                                                   '(67 44
                                                                     189))))
                                                             (GRID:COPY-TO
                                                              (ELT* V1
                                                                   1.39d0)
							       'array '(unSIGNED-BYTE 32)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(-88 -94 98))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     64)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71))))
                                                             (GRID:COPY-TO
                                                              (ELT* V1
                                                                   1.39d0)
							       'array '(SIGNED-BYTE 64)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(93 61 262))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(UNSIGNED-BYTE
                                                                     64)
                                                                   :INITIAL-CONTENTS
                                                                   '(67 44
                                                                     189))))
                                                             (GRID:COPY-TO
                                                              (ELT* V1
                                                                   1.39d0)
							       'array '(unSIGNED-BYTE 64))))))

