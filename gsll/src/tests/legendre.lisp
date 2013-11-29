;; Regression test LEGENDRE for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST LEGENDRE
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST 0.3d0 0.0d0)
                                                          (MULTIPLE-VALUE-LIST
                                                           (LEGENDRE-P1
                                                            0.3d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.365d0 2.8199664825478977d-16)
                        (MULTIPLE-VALUE-LIST (LEGENDRE-P2 0.3d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.38249999999999995d0 1.9984014443252816d-16)
                        (MULTIPLE-VALUE-LIST (LEGENDRE-P3 0.3d0)))
                       (LISP-UNIT:ASSERT-ERROR 'INPUT-DOMAIN
                                               (LEGENDRE-PL -4 0.3d0))
                       (LISP-UNIT:ASSERT-ERROR 'INPUT-DOMAIN
                                               (LEGENDRE-PL 4 3.0d0))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.07293749999999999d0 1.0167387765047662d-16)
                        (MULTIPLE-VALUE-LIST (LEGENDRE-PL 4 0.3d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(1.0d0 0.5d0 -0.125d0 -0.4375d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((ARR (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS 4)))
                           (LEGENDRE-PL-ARRAY 0.5d0 ARR)
                           (GRID:COPY-TO ARR))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.3128529498822064d0 1.3893461931245028d-16)
                        (MULTIPLE-VALUE-LIST (LEGENDRE-Q0 3.3d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.03241473461128108d0 1.4395033881023292d-17)
                        (MULTIPLE-VALUE-LIST (LEGENDRE-Q1 3.3d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.004026461384737812d0 1.788108054840004d-18)
                        (MULTIPLE-VALUE-LIST (LEGENDRE-QL 2 3.3d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -34.099750274012266d0 3.0286662310541114d-14)
                        (MULTIPLE-VALUE-LIST (LEGENDRE-PLM 4 3 0.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(2.25d0 5.625d0 4.21875d0 -4.921875d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((ARR (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS 4)))
                           (LEGENDRE-PLM-ARRAY 2 0.5d0 ARR)
                           (GRID:COPY-TO ARR))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST #(-3.0d0 3.75d0 33.75d0 55.78125d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((VAL (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS 4))
                               (DERIV
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS 4)))
                           (LEGENDRE-PLM-DERIV-ARRAY 2 0.5d0 VAL DERIV)
                           (GRID:COPY-TO DERIV))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.30366280894310793d0 3.5267592419993454d-14)
                        (MULTIPLE-VALUE-LIST
                         (LEGENDRE-SPHPLM 1200 1100 0.3d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #(0.24892463950030283d0 0.4127948151484927d0
                           0.35120655562190445d0 0.051599351893561574d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((ARR (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS 4)))
                           (LEGENDRE-SPHPLM-ARRAY 4 0.5d0 ARR)
                           (GRID:COPY-TO ARR))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #(-0.6637990386674741d0 -0.2751965434323283d0
                           1.2710332489173686d0 2.648766730536161d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((VAL (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS 4))
                               (DERIV
                                (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS 4)))
                           (LEGENDRE-SPHPLM-DERIV-ARRAY 4 0.5d0 VAL DERIV)
                           (GRID:COPY-TO DERIV))))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.1255299048878925d0 1.3395992025650077d-15)
                        (MULTIPLE-VALUE-LIST
                         (LEGENDRE-CONICALP-HALF 3.5d0 10.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.06274336292793128d0 2.504525777730328d-16)
                        (MULTIPLE-VALUE-LIST
                         (LEGENDRE-CONICALP-MHALF 3.5d0 10.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -0.1316366189368757d0 3.0865532615549887d-15)
                        (MULTIPLE-VALUE-LIST
                         (LEGENDRE-CONICALP-0 3.5d0 10.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.1740719556007628d0 4.02431272707091d-15)
                        (MULTIPLE-VALUE-LIST
                         (LEGENDRE-CONICALP-1 3.5d0 10.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 8.980797952969897d-4 7.157154480497032d-17)
                        (MULTIPLE-VALUE-LIST
                         (LEGENDRE-REGULAR-SPHERICAL-CONICAL 3 3.5d0 10.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.0023060250619859808d0 2.7345630541297113d-16)
                        (MULTIPLE-VALUE-LIST
                         (LEGENDRE-REGULAR-CYLINDRICAL-CONICAL 3 3.5d0
                                                               10.0d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.9200342692589382d0 1.7018240558144874d-15)
                        (MULTIPLE-VALUE-LIST (LEGENDRE-H3D-0 1.0d0 0.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.2169402645039212d0 1.418962379417407d-15)
                        (MULTIPLE-VALUE-LIST (LEGENDRE-H3D-1 1.0d0 0.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 0.002400616238997978d0 4.3381136468045d-17)
                        (MULTIPLE-VALUE-LIST (LEGENDRE-H3D 4 1.0d0 0.5d0)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST
                         #(0.9200342692589379d0 0.21694026450392115d0
                           0.04795066048830776d0 0.010663769096144337d0))
                        (MULTIPLE-VALUE-LIST
                         (LET ((ARR (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :DIMENSIONS 4)))
                           (LEGENDRE-H3D-ARRAY 1.0d0 0.5d0 ARR)
                           (GRID:COPY-TO ARR)))))

