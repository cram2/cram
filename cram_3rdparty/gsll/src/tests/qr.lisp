;; Regression test QR for GSLL, automatically generated
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
(LISP-UNIT:DEFINE-TEST QR
  ;; QR solve
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (list *hilb2-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-SOLVE-DIM *HILB2*))))
  (let ((lisp-unit:*epsilon* (* 2 128 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb3-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-SOLVE-DIM *HILB3*))))
  (let ((lisp-unit:*epsilon* (* 2 2048 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb4-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-SOLVE-DIM *HILB4*))))
  (let ((lisp-unit:*epsilon* 0.5d0))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb12-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-SOLVE-DIM *HILB12*))))
  (let ((lisp-unit:*epsilon* (* 2 8 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander2-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-SOLVE-DIM *VANDER2*))))
  (let ((lisp-unit:*epsilon* (* 2 64 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander3-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-SOLVE-DIM *vander3*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander4-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-SOLVE-DIM *vander4*))))
  (let ((lisp-unit:*epsilon* 0.05d0))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander12-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-SOLVE-DIM *vander12*))))
  ;; QR QRsolve
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (list *hilb2-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-QRSOLVE-DIM *HILB2*))))
  (let ((lisp-unit:*epsilon* (* 2 128 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (list *hilb3-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-QRSOLVE-DIM *HILB3*))))
  (let ((lisp-unit:*epsilon* (* 2 2048 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (list *hilb4-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-QRSOLVE-DIM *HILB4*))))
  (let ((lisp-unit:*epsilon* 0.5d0))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (list *hilb12-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-QRSOLVE-DIM *HILB12*))))
  (let ((lisp-unit:*epsilon* (* 2 8 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander2-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-QRSOLVE-DIM *VANDER2*))))
  (let ((lisp-unit:*epsilon* (* 2 64 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander3-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-QRSOLVE-DIM *vander3*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander4-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-QRSOLVE-DIM *vander4*))))
  (let ((lisp-unit:*epsilon* 0.05d0))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander12-soln*)
     (MULTIPLE-VALUE-LIST (TEST-QR-QRSOLVE-DIM *vander12*))))
  ;; QR LSsolve
  (let ((lisp-unit:*epsilon* (* 2 128 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST
      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
		   '(52.5992295702070d0 -337.7263113752073d0 351.8823436427604d0))
      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
		   '(-0.03018843019022285d0
		     0.30523642927546163d0
		     -0.4779674081526012d0
		     -0.25160239130369944d0
		     0.4989664138130865d0)))
     (MULTIPLE-VALUE-LIST (TEST-QR-LSSOLVE-DIM *M53*))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST
      *hilb2-soln*
      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
		   '(0.0d0 0.0d0)))
     (MULTIPLE-VALUE-LIST (TEST-QR-LSSOLVE-DIM *HILB2*))))
  (let ((lisp-unit:*epsilon* (* 2 128 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST
      *hilb3-soln*
      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
		   '(0.0d0 0.0d0 0.0d0)))
     (MULTIPLE-VALUE-LIST (TEST-QR-LSSOLVE-DIM *HILB3*))))
  (let ((lisp-unit:*epsilon* (* 2 2048 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST
      *hilb4-soln*
      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
		   '(0.0d0 0.0d0 0.0d0 0.0d0)))
     (MULTIPLE-VALUE-LIST (TEST-QR-LSSOLVE-DIM *HILB4*))))
  (let ((lisp-unit:*epsilon* 0.5d0))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST
      *hilb12-soln*
      (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
		   '(0.0d0 0.0d0 0.0d0 0.0d0 0.0d0 0.0d0
		     0.0d0 0.0d0 0.0d0 0.0d0 0.0d0 0.0d0)))
     (MULTIPLE-VALUE-LIST (TEST-QR-LSSOLVE-DIM *HILB12*))))
  (let ((lisp-unit:*epsilon* (* 2 8 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander2-soln*
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
			'(0.0d0 0.0d0)))
     (MULTIPLE-VALUE-LIST (TEST-QR-LSSOLVE-DIM *VANDER2*))))
  (let ((lisp-unit:*epsilon* (* 2 64 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander3-soln*
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
			'(0.0d0 0.0d0 0.0d0)))
     (MULTIPLE-VALUE-LIST (TEST-QR-LSSOLVE-DIM *vander3*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander4-soln*
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
			'(0.0d0 0.0d0 0.0d0 0.0d0)))
     (MULTIPLE-VALUE-LIST (TEST-QR-LSSOLVE-DIM *vander4*))))
  (let ((lisp-unit:*epsilon* 0.05d0))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander12-soln*
	   (GRID:MAKE-FOREIGN-ARRAY 'DOUBLE-FLOAT :INITIAL-CONTENTS
			'(0.0d0 0.0d0 0.0d0 0.0d0 0.0d0 0.0d0
			  0.0d0 0.0d0 0.0d0 0.0d0 0.0d0 0.0d0)))
     (MULTIPLE-VALUE-LIST (TEST-QR-LSSOLVE-DIM *vander12*))))
  ;; QR decomp
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *m35*)
     (MULTIPLE-VALUE-LIST (TEST-QR-DECOMP-DIM *M35*))))
  (let ((lisp-unit:*epsilon* (* 2 128 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *m53*)
     (MULTIPLE-VALUE-LIST (TEST-QR-DECOMP-DIM *M53*))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb2*)
     (MULTIPLE-VALUE-LIST (TEST-QR-DECOMP-DIM *HILB2*))))
  (let ((lisp-unit:*epsilon* (* 2 128 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb3*)
     (MULTIPLE-VALUE-LIST (TEST-QR-DECOMP-DIM *HILB3*))))
  (let ((lisp-unit:*epsilon* (* 2 2048 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb4*)
     (MULTIPLE-VALUE-LIST (TEST-QR-DECOMP-DIM *HILB4*))))
  (let ((lisp-unit:*epsilon* (* 2 2048 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *hilb12*)
     (MULTIPLE-VALUE-LIST (TEST-QR-DECOMP-DIM *HILB12*))))
  (let ((lisp-unit:*epsilon* (* 2 8 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander2*)
     (MULTIPLE-VALUE-LIST (TEST-QR-DECOMP-DIM *vander2*))))
  (let ((lisp-unit:*epsilon* (* 2 64 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander3*)
     (MULTIPLE-VALUE-LIST (TEST-QR-DECOMP-DIM *vander3*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander4*)
     (MULTIPLE-VALUE-LIST (TEST-QR-DECOMP-DIM *vander4*))))
  (let ((lisp-unit:*epsilon* 0.0005d0))	; "FIXME: bad accuracy"
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST *vander12*)
     (MULTIPLE-VALUE-LIST (TEST-QR-DECOMP-DIM *VANDER12*))))
  ;; QR update
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST (constant-matrix 0 3 5))
     (MULTIPLE-VALUE-LIST (TEST-QR-UPDATE-DIM *M35*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST (constant-matrix 0 5 3))
     (MULTIPLE-VALUE-LIST (TEST-QR-UPDATE-DIM *M53*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST (constant-matrix 0 2))
     (MULTIPLE-VALUE-LIST (TEST-QR-UPDATE-DIM *HILB2*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST (constant-matrix 0 3))
     (MULTIPLE-VALUE-LIST (TEST-QR-UPDATE-DIM *HILB3*))))
  (let ((lisp-unit:*epsilon* (* 2 2048 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST (constant-matrix 0 4))
     (MULTIPLE-VALUE-LIST (TEST-QR-UPDATE-DIM *HILB4*))))
  (let ((lisp-unit:*epsilon* (* 2 2048 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST (constant-matrix 0 12))
     (MULTIPLE-VALUE-LIST (TEST-QR-UPDATE-DIM *HILB12*))))
  (let ((lisp-unit:*epsilon* (* 2 8 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST (constant-matrix 0 2))
     (MULTIPLE-VALUE-LIST (TEST-QR-UPDATE-DIM *vander2*))))
  (let ((lisp-unit:*epsilon* (* 2 64 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST (constant-matrix 0 3))
     (MULTIPLE-VALUE-LIST (TEST-QR-UPDATE-DIM *vander3*))))
  (let ((lisp-unit:*epsilon* (* 2 1024 double-float-epsilon)))
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST (constant-matrix 0 4))
     (MULTIPLE-VALUE-LIST (TEST-QR-UPDATE-DIM *vander4*))))
  (let ((lisp-unit:*epsilon* 0.0005d0))	; "FIXME: bad accuracy"
    (LISP-UNIT:ASSERT-NUMERICAL-EQUAL
     (LIST (constant-matrix 0 12))
     (MULTIPLE-VALUE-LIST (TEST-QR-UPDATE-DIM *VANDER12*)))))
