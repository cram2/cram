;; Series acceleration.
;; Liam Healy, Wed Nov 21 2007 - 18:41
;; Time-stamp: <2010-06-30 19:57:28EDT series-acceleration.lisp>
;;
;; Copyright 2007, 2008, 2009 Liam M. Healy
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

;;; /usr/include/gsl/gsl_sum.h

;;;;****************************************************************************
;;;; Creation and calculation of Levin series acceleration
;;;;****************************************************************************

(defun levin-value (levin slot)
  (cffi:foreign-slot-value (mpointer levin) 'levin-c slot))

(defmobject levin "gsl_sum_levin_u"
  ((order sizet))
  "Levin u-transform"
  :documentation			; FDL
  "Make a workspace for a Levin u-transform of n
   terms.  The size of the workspace is O(2n^2 + 3n).")

(defmfun accelerate (array levin)
  "gsl_sum_levin_u_accel"
  (((foreign-pointer array) :pointer) ((dim0 array) sizet) ((mpointer levin) :pointer)
   (accelerated-sum (:pointer :double)) (abserr (:pointer :double)))
  :inputs (array)
  :documentation			; FDL
  "From the terms of a series in array, compute the extrapolated
   limit of the series using a Levin u-transform.  Additional
   working space must be provided in levin.  The extrapolated sum is
   returned with an estimate of the absolute error.  The actual
   term-by-term sum is returned in 
   w->sum_plain. The algorithm calculates the truncation error
   (the difference between two successive extrapolations) and round-off
   error (propagated from the individual terms) to choose an optimal
   number of terms for the extrapolation.")

;;;;****************************************************************************
;;;; Acceleration with error estimation from truncation
;;;;****************************************************************************

(defmobject levin-truncated "gsl_sum_levin_utrunc"
  ((order sizet))
  "truncated Levin u-transform"
  :documentation			; FDL
  "Make a workspace for a Levin u-transform of n
   terms, without error estimation.  The size of the workspace is
   O(3n).")

(defmfun accelerate-truncated (array levin)
  "gsl_sum_levin_utrunc_accel"
  (((foreign-pointer array) :pointer) ((dim0 array) sizet) ((mpointer levin) :pointer)
   (accelerated-sum (:pointer :double)) (abserr (:pointer :double)))
  :inputs (array)
  :documentation			; FDL
  "From the terms of a series in array, compute the extrapolated
   limit of the series using a Levin u-transform.  Additional
   working space must be provided in levin.  The extrapolated sum is
   returned with an estimate of the absolute error.  The actual
   term-by-term sum is returned in w->sum_plain. The algorithm
   terminates when the difference between two successive extrapolations
   reaches a minimum or is sufficiently small. The difference between
   these two values is used as estimate of the error and is stored in
   abserr_trunc.  To improve the reliability of the algorithm the
   extrapolated values are replaced by moving averages when calculating
   the truncation error, smoothing out any fluctuations.")

;;;;****************************************************************************
;;;; Example
;;;;****************************************************************************

;;; From Sec. 29.3 in the GSL manual.

(defun acceleration-example (&optional (print-explanation t))
  (let ((maxterms 20)
	(sum 0.0d0)
	(zeta2 (/ (expt pi 2) 6)))
    (let ((levin (make-levin maxterms))
	  (array (grid:make-foreign-array 'double-float :dimensions maxterms)))
      (dotimes (n maxterms)
	(setf (grid:gref array n) (coerce (/ (expt (1+ n) 2)) 'double-float))
	(incf sum (grid:gref array n)))
      (multiple-value-bind (accelerated-sum error)
	  (accelerate array levin)
	(when print-explanation
	  (format t "term-by-term sum =~f using ~d terms~&" sum maxterms)
	  (format t "term-by-term sum =~f using ~d terms~&"
		  (levin-value levin 'sum-plain)
		  (levin-value levin 'terms-used))
	  (format t "exact value     = ~f~&" zeta2)
	  (format t "accelerated sum = ~f using ~d terms~&"
		  accelerated-sum
		  (levin-value levin 'terms-used))
	  (format t "estimated error = ~f~&" error)
	  (format t "actual error = ~f ~&" (- accelerated-sum zeta2)))
	(values sum maxterms
		(levin-value levin 'sum-plain)
		(levin-value levin 'terms-used)
		accelerated-sum
		error
		(- accelerated-sum zeta2))))))

;; term-by-term sum =1.5961632439130233 using 20 terms
;; term-by-term sum =1.5759958390005426 using 13 terms
;; exact value     = 1.6449340668482264
;; accelerated sum = 1.6449340669228176 using 13 terms
;; estimated error = 0.00000000008883604962761638
;; actual error = 0.00000000007459122208786084

(save-test series-acceleration (acceleration-example nil))
