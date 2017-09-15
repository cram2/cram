;; Regression test SVD for GSLL, automatically generated
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

;;; Answers inserted from linalg/test.c
;;; GSL has #define GSL_DBL_EPSILON        2.2204460492503131e-16
;;; which is 2x what double-float-epsilon is.
(LISP-UNIT:DEFINE-TEST SVD
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb2-soln*)
     (MULTIPLE-VALUE-LIST
      (TEST-SV-SOLVE-DIM *HILB2*))))
  (let ((lisp-unit:*epsilon* (* 2 128 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb3-soln*)
     (MULTIPLE-VALUE-LIST
      (TEST-SV-SOLVE-DIM *HILB3*))))
  (let ((lisp-unit:*epsilon* (* 2 2048 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb4-soln*)
     (MULTIPLE-VALUE-LIST (TEST-SV-SOLVE-DIM *hilb4*))))
  (let ((lisp-unit:*epsilon* 0.5d0))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb12-soln*)
     (MULTIPLE-VALUE-LIST (TEST-SV-SOLVE-DIM *hilb12*))))
  (let ((lisp-unit:*epsilon* (* 2 64 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander2-soln*)
     (MULTIPLE-VALUE-LIST (TEST-SV-SOLVE-DIM *vander2*))))
  (let ((lisp-unit:*epsilon* (* 2 64 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander3-soln*)
     (MULTIPLE-VALUE-LIST (TEST-SV-SOLVE-DIM *vander3*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander4-soln*)
     (MULTIPLE-VALUE-LIST (TEST-SV-SOLVE-DIM *vander4*))))
  (let ((lisp-unit:*epsilon* 0.05d0))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander12-soln*)
     (MULTIPLE-VALUE-LIST (TEST-SV-SOLVE-DIM *vander12*)))))
