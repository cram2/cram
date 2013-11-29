;; Regression test ELLIPTIC-FUNCTIONS for GSLL
;;
;; Copyright 2009, 2010 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST
    ELLIPTIC-FUNCTIONS
  (let ((lisp-unit:*epsilon* (* 2 2 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 0.4707504736556572833d0 0.8822663948904402865d0 0.9429724257773856873d0)
     (multiple-value-list (jacobian-elliptic-functions 0.5d0 0.5d0))))
  (let ((lisp-unit:*epsilon* (* 2 2 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 0.8187707145344889190d0 0.5741206467465548795d0 0.8938033089590823040d0)
     (multiple-value-list (jacobian-elliptic-functions 1.0d0 0.3d0))))
  (let ((lisp-unit:*epsilon* (* 2 2 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 0.7949388393365780943d0 0.6066895760718277578d0 0.7879361300438814425d0)
     (multiple-value-list (jacobian-elliptic-functions 1.0d0 0.6d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 0.96402778575700186570d0 0.26580148285600686381d0 0.26580323105264131136d0)
     (multiple-value-list (jacobian-elliptic-functions 2.0d0 0.999999d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     ;; The first one should actually be TOL0
     (list 1.0d0 0.0d0 0.8541791304497336d0)
     (multiple-value-list (jacobian-elliptic-functions 1.69695970624443d0 0.270378013104138d0))))
  (let ((lisp-unit:*epsilon* (* 2 2 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 0.0d0 1.0d0 1.0d0)
     (multiple-value-list (jacobian-elliptic-functions 0.0d0 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 2 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list -1.0d-10 1.0d0 1.0d0)
     (multiple-value-list  (jacobian-elliptic-functions -1.0d-10 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 2 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 1.0d-10 1.0d0 1.0d0)
     (multiple-value-list (jacobian-elliptic-functions 1.0d-10 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 2 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 1.0d-30 1.0d0 1.0d0)
     (multiple-value-list (jacobian-elliptic-functions 1.0d-30 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 256 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list (- *elljac-A* (* 1.0d-10 *elljac-B* *elljac-C*))
	   (+ *elljac-B* (* 1.0d-10 *elljac-A* *elljac-C*))
	   (+ *elljac-C* (* 0.1d0 1.0d-10 *elljac-A* *elljac-B*)))
     (multiple-value-list (jacobian-elliptic-functions (- (/ *elljac-K* 2.0d0) 1.0d-10) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 256 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list *elljac-A* *elljac-B* *elljac-C*)
     (multiple-value-list (jacobian-elliptic-functions (/ *elljac-K* 2.0d0) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list (+ *elljac-A* (* 1.0d-10 *elljac-B* *elljac-C*))
	   (- *elljac-B* (* 1.0d-10 *elljac-A* *elljac-C*))
	   (- *elljac-C* (* 0.1d0 1.0d-10 *elljac-A* *elljac-B*)))
     (multiple-value-list (jacobian-elliptic-functions (+ (/ *elljac-K* 2.0d0) 1.0d-10) 0.1d0))))
  (let ((lisp-unit:*epsilon* 1.0d-5)) ; Varying tolerances, max = 10*TEST_SNGL
    (lisp-unit::assert-numerical-equal 
     (list 1.0d0 (* 1.0d-10 *elljac-C2*) *elljac-C2* )
     (multiple-value-list (jacobian-elliptic-functions (- *elljac-K* 1.0d-10) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 256 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 1.0d0 0.0d0 *elljac-C2*)
     (multiple-value-list (jacobian-elliptic-functions *elljac-K* 0.1d0))))
  (let ((lisp-unit:*epsilon* 1.0d-5)) ; Varying tolerances, max = 10*TEST_SNGL
    (lisp-unit::assert-numerical-equal
     (list 1.0d0 (* -1.0d-10 *elljac-C2*) *elljac-C2*)
     (multiple-value-list (jacobian-elliptic-functions (+ *elljac-K* 1.0d-10) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 256 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list *elljac-A* (- *elljac-B*) *elljac-C*)
     (multiple-value-list (jacobian-elliptic-functions (* 3/2 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* 1.0d-5)) ; Varying tolerances, max = 10*TEST_SNGL
    (lisp-unit::assert-numerical-equal
     (list 1.0d-10 -1.0d0 1.0d0)
     (multiple-value-list (jacobian-elliptic-functions (- (* 2 *elljac-K*) 1.0d-10) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 0.0d0 -1.0d0 1.0d0)
     (multiple-value-list (jacobian-elliptic-functions (* 2 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* 1.0d-5)) ; Varying tolerances, max = 10*TEST_SNGL
    (lisp-unit::assert-numerical-equal
     (list -1.0d-10 -1.0d0 1.0d0)
     (multiple-value-list (jacobian-elliptic-functions (+ (* 2 *elljac-K*) 1.0d-10) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list (- *elljac-A*) (- *elljac-B*) *elljac-C*)
     (multiple-value-list (jacobian-elliptic-functions (* 5/2 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* 1.0d-5)) ; Varying tolerances, max = 10*TEST_SNGL
    (lisp-unit::assert-numerical-equal
     (list -1.0d0 (* -1.0d-10 *elljac-C2*) *elljac-C2*)
     (multiple-value-list (jacobian-elliptic-functions (- (* 3 *elljac-K*) 1.0d-10) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list -1.0d0 0.0d0 *elljac-C2*)
     (multiple-value-list (jacobian-elliptic-functions (* 3 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* 1.0d-5)) ; Varying tolerances, max = 10*TEST_SNGL
    (lisp-unit::assert-numerical-equal
     (list -1.0d0 (* 1.0d-10 *elljac-C2*) *elljac-C2*)
     (multiple-value-list (jacobian-elliptic-functions (+ (* 3 *elljac-K*) 1.0d-10) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list (- *elljac-A*) *elljac-B* *elljac-C*)
     (multiple-value-list (jacobian-elliptic-functions (* 7/2 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* 1.0d-5)) ; Varying tolerances, max = 10*TEST_SNGL
    (lisp-unit::assert-numerical-equal
     (list -1.0d-10 1.0d0 1.0d0)
     (multiple-value-list (jacobian-elliptic-functions (- (* 4 *elljac-K*) 1.0d-10) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 0.0d0 1.0d0 1.0d0)
     (multiple-value-list (jacobian-elliptic-functions (* 4 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list *elljac-A* *elljac-B* *elljac-C*)
     (multiple-value-list (jacobian-elliptic-functions (* 9/2 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list (- *elljac-A*) *elljac-B* *elljac-C*)
     (multiple-value-list (jacobian-elliptic-functions (* -1/2 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list -1.0d0 0.0d0 *elljac-C2*)
     (multiple-value-list (jacobian-elliptic-functions (* -1 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list (- *elljac-A*) (- *elljac-B*) *elljac-C*)
     (multiple-value-list (jacobian-elliptic-functions (* -3/2 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 2 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 0.0d0 -1.0d0 1.0d0)
     (multiple-value-list (jacobian-elliptic-functions (* -2 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list *elljac-A* (- *elljac-B*) *elljac-C*)
     (multiple-value-list (jacobian-elliptic-functions (* -5/2 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 1.0d0 0.0d0 *elljac-C2*)
     (multiple-value-list (jacobian-elliptic-functions (* -3 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 16 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list *elljac-A* *elljac-B* *elljac-C*)
     (multiple-value-list (jacobian-elliptic-functions (* -7/2 *elljac-K*) 0.1d0))))
  (let ((lisp-unit:*epsilon* (* 2 2 double-float-epsilon)))
    (lisp-unit::assert-numerical-equal
     (list 0.0d0 1.0d0 1.0d0)
     (multiple-value-list (jacobian-elliptic-functions (* -4 *elljac-K*) 0.1d0))))
  (LISP-UNIT:ASSERT-ERROR 'INPUT-DOMAIN (JACOBIAN-ELLIPTIC-FUNCTIONS 0.61802d0 1.5d0)))

