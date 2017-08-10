;; GSL defined tests
;; Liam Healy 2014-01-21 10:42:48EST gsl-tests.lisp
;; Time-stamp: <2014-02-09 17:47:37EST gsl-tests.lisp>

;; Copyright 2014 Liam M. Healy
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

;;; GSL test functions are defined in gsl-1.16/test/results.i.

;;; Need constants in ~/mathematics/gsl-1.16/gsl_machine.h

(defparameter *gsl-test-verbose* t)

(defun gsl-test-rel
    (result expected relative-error test-description &optional (verbose *gsl-test-verbose*))
  ;; Instead of "status" integer, we return T for passing and a symbol for the category of failure.
  (let ((status
	  (cond ((or (nanp result) (nanp expected))
		 (or (eql (nanp result) (nanp expected)) :nan))
		((or (infinityp  result) (infinityp expected))
		 (or (eql (infinityp  result) (infinityp expected)) :infinity))
		((or (and (plusp expected) (< expected +DBL-MIN+))
		     (and (minusp expected) (> expected (- +DBL-MIN+))))
		 :double-float-range)
		((not (zerop expected))
		 (or (<= (/ (abs (- result expected)) (abs expected)) relative-error)
		     :nonzero-relative-error))
		(t (or (<= (abs result) relative-error)
		       :absolute-error)))))
    (if (or verbose (not (eql status t)))
	(if (eql status t)
	    (format t "PASS~8t~a~8t~g observed vs. ~g expected" test-description result expected)
	    (format t
		    "FAIL~8t~a~8t~a, ~g observed vs. ~g expected"
		    status test-description result expected)))))


;;;    TEST(gsl_cdf_beta_P, (0.0000000000000000e+00,1.3,2.7), 0.000000000000e+00, TEST_TOL6);
;;; (convert-gsl-form )

;; gsl-lookup("gsl_cdf_beta_P")

;; (gsl-test-rel (BETA-P 0.0000000000000000d+00 1.3d0 2.7d0) 0.000000000000d+00 +TEST-TOL6+ "(BETA-P 0.0000000000000000d+00 1.3d0 2.7d0)" t)
