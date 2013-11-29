;; Additional methods for lisp-unit
;; Liam Healy 2009-04-15 23:23:30EDT augment.lisp
;; Time-stamp: <2010-07-07 14:24:56EDT augment.lisp>
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

(defmethod lisp-unit:numerical-equal
    ((result1 grid:foreign-array) (result2 grid:foreign-array) &key (test #'lisp-unit:number-equal))
  "Return true if the arrays are numerically equal according to :TEST."
  (when (equal (dimensions result1) (dimensions result2))
    (lisp-unit:numerical-equal (grid:copy-to result1) (grid:copy-to result2)
			       :test test)))

;;; See cdf/test.c
(defconstant +test-tol0+ (* 2 +dbl-epsilon+))
(defconstant +test-tol1+ (* 16 +dbl-epsilon+))
(defconstant +test-tol2+ (* 256 +dbl-epsilon+))
(defconstant +test-tol3+ (* 2048 +dbl-epsilon+))
(defconstant +test-tol4+ (* 16384 +dbl-epsilon+))
(defconstant +test-tol5+ (* 131072 +dbl-epsilon+))
(defconstant +test-tol6+ (* 1048576 +dbl-epsilon+))
(defconstant +test-sqrt-tol0+ (* 2 +sqrt-dbl-epsilon+))

;;; These are 1.0 if not "RELEASED"
(defconstant +test-sigma+ 1.5d0)
(defconstant +test-factor+ 100.0d0)

(defun sf-frac-diff (x1 x2)
  ;; After test_sf_frac_diff in specfunc/test_sf.c.
  (cond ((and (zerop x1) (zerop x2))
	 (coerce 0 (type-of x1)))
	((zerop x1)
	 (abs x2))
	((and (<= x1 most-positive-double-float)
	      (<= x2 most-positive-double-float)
	      (not (zerop (+ x1 x2))))
	 (abs (/ (- x1 x2) (+ x1 x2))))
	(t 1.0d0)))

(defun sf-check-single (result expected-value tolerance &optional error-estimate)
  "Check the result of a single value as in test_sf_check_result in specfunc/test_sf.c." 
  (or (eql result expected-value) ; catch expected inifinity/nan
      (let ((diff (abs (- result expected-value))))
	(and
	 (<= (sf-frac-diff expected-value result) (* +test-factor+ tolerance))
	 (if error-estimate
	     (and (not (minusp error-estimate)) ; redundant but signalled as separate error in C
		  (finitep error-estimate)
		  (<= diff (* 2 +test-sigma+ error-estimate)))
	     t)))))

(defun sf-check-results (result-list expected-value tolerance)
  "Check the results of multiple returns where each value may have an
   error estimate returned as well."
  ;; After test_sf_check_result in specfunc/test_sf.c.
  (when (atom expected-value)
    (setf expected-value (list expected-value)))
  (let ((errorsp (= (length result-list) (* 2 (length expected-value)))))
    ;; Error information is returned
    (loop for ind below (length expected-value)
       always
       (sf-check-single
	(elt result-list ind)
	(elt expected-value ind)
	tolerance
	(when errorsp
	  (elt result-list (+ ind (length expected-value))))))))

;; (assert-to-tolerance (tdist-P 0.0d0 1.0d0) 0.5d0 +test-tol6+)
(defmacro assert-to-tolerance (form expected-value tolerance)
  ;; Equivalent of TEST_SF.
  `(lisp-unit::assert-true
    (sf-check-results
     (multiple-value-list ,form) ,expected-value ,tolerance)))

  ;; and TEST_SF becomes assert-to-tolerance.
(defmacro assert-sf-scale (form expected-value scale result-tol &optional err-tol)
  ;; Equivalent of TEST_SF_E10.
  ;; Some of the tests in test_exp like gsl_sf_exp_mult_e10_e(1.0,
  ;; 1.0, &re) do a check of test_sf_frac_diff, then of the err, then
  ;; the scale (e10), then they (redundantly?) call test_sf_e10.
  ;; Others just call test_sf_e10.  This attempts to cover both cases.
  `(lisp-unit::assert-true
    (multiple-value-bind (val scale err)
	,form
      (and
       (sf-check-single val ,expected-value ,result-tol err)
       (= scale ,scale)
       ,(if err-tol `(<= err ,err-tol) t)))))

(defmacro assert-posinf (form)
  `(lisp-unit::assert-true
    (let ((val ,form))
      (and (infinityp val) (plusp val)))))

(defmacro assert-neginf (form)
  `(lisp-unit::assert-true
    (let ((val ,form))
      (and (infinityp val) (minusp val)))))
