;; Regression test VECTOR-SUB for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST VECTOR-SUB
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(-25.57 -25.88 9.440001))
                        (MULTIPLE-VALUE-LIST
                         (LET ((V1
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
                                             '(-34.5 8.24 3.29)))
                               (V2
                                (GRID:MAKE-FOREIGN-ARRAY 'SINGLE-FLOAT :INITIAL-CONTENTS
                                             '(-8.93 34.12 -6.15))))
                           (GRID:COPY-TO (ELT- V1 V2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #(-25.57d0 -25.879999999999995d0 9.440000000000001d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((V1
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '(-34.5d0 8.24d0 3.29d0)))
                               (V2
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
                                             '(-8.93d0 34.12d0 -6.15d0))))
                           (GRID:COPY-TO (ELT- V1 V2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(27 -120 81))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     8)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71)))
                                                                 (V2
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     8)
                                                                   :INITIAL-CONTENTS
                                                                   '(-91 52
                                                                     -10))))
                                                             (GRID:COPY-TO
                                                              (ELT- V1 V2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(207 137 49))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(UNSIGNED-BYTE
                                                                     8)
                                                                   :INITIAL-CONTENTS
                                                                   '(67 44
                                                                     189)))
                                                                 (V2
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(UNSIGNED-BYTE
                                                                     8)
                                                                   :INITIAL-CONTENTS
                                                                   '(116 163
                                                                     140))))
                                                             (GRID:COPY-TO
                                                              (ELT- V1 V2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(27 -120 81))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     16)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71)))
                                                                 (V2
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     16)
                                                                   :INITIAL-CONTENTS
                                                                   '(-91 52
                                                                     -10))))
                                                             (GRID:COPY-TO
                                                              (ELT- V1 V2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(65487 65417 49))
                        (MULTIPLE-VALUE-LIST
                         (LET ((V1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 16)
                                             :INITIAL-CONTENTS '(67 44 189)))
                               (V2
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 16)
                                             :INITIAL-CONTENTS '(116 163 140))))
                           (GRID:COPY-TO (ELT- V1 V2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(27 -120 81))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     32)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71)))
                                                                 (V2
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     32)
                                                                   :INITIAL-CONTENTS
                                                                   '(-91 52
                                                                     -10))))
                                                             (GRID:COPY-TO
                                                              (ELT- V1 V2)))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(4294967247 4294967177 49))
                        (MULTIPLE-VALUE-LIST
                         (LET ((V1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 32)
                                             :INITIAL-CONTENTS '(67 44 189)))
                               (V2
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 32)
                                             :INITIAL-CONTENTS '(116 163 140))))
                           (GRID:COPY-TO (ELT- V1 V2)))))
		       #+int64
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST #(27 -120 81))
                                                          (MULTIPLE-VALUE-LIST
                                                           (LET ((V1
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     64)
                                                                   :INITIAL-CONTENTS
                                                                   '(-64 -68
                                                                     71)))
                                                                 (V2
                                                                  (GRID:MAKE-FOREIGN-ARRAY
                                                                   '(SIGNED-BYTE
                                                                     64)
                                                                   :INITIAL-CONTENTS
                                                                   '(-91 52
                                                                     -10))))
                                                             (GRID:COPY-TO
                                                              (ELT- V1 V2)))))
		       #+int64
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(18446744073709551567 18446744073709551497 49))
                        (MULTIPLE-VALUE-LIST
                         (LET ((V1
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 64)
                                             :INITIAL-CONTENTS '(67 44 189)))
                               (V2
                                (GRID:MAKE-FOREIGN-ARRAY '(UNSIGNED-BYTE 64)
                                             :INITIAL-CONTENTS '(116 163 140))))
                           (GRID:COPY-TO (ELT- V1 V2))))))

