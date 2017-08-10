;; Sample sets of values from a low-discrepancy (quasi-random) sequence
;; Liam Healy 2011-06-06 09:42:22EDT low-discrepancy-sequence.lisp
;; Time-stamp: <2015-11-22 12:36:08EST low-discrepancy-sequence.lisp>

;; Copyright 2011, 2013, 2014, 2015 Liam M. Healy
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

(in-package :antik)

(export '(low-discrepancy-sample apply-to-arguments list-no-key))

(defun val-from-range (val min max)
  (+ min (* val (- max min))))

(defun quasi-random-values
    (rank count &optional (rng-type gsl:+sobol+) (grid-type grid:*default-grid-type*))
  "Generate a set of count values evenly distributed across the range [0,1] for each dimension."
  (let ((gen (gsl:make-quasi-random-number-generator rng-type rank))
	(vec (grid:make-foreign-array 'double-float :dimensions rank)))
    (loop repeat count
	  do (gsl:qrng-get gen vec)
	  collect (grid:copy vec :grid-type grid-type :element-type 'double-float))))

(defun list-no-key (&rest args)
  "Make a list of the values without the keywords."
  (loop for (key val) on args by #'cddr collect val))

(defun low-discrepancy-sample (count make-function &rest parameters)
  "Call the make function to generate a set of objects with the parameters sampled according to a low-discrepancy sequence.  Each parameter will be either a fixed value, specified as a list of key and value, or a range from which the sample is taken, specified as a list of key, lower value, and upper value."
  (let* ((fixed-parameters
	  (apply 'append
		 (remove-if-not (lambda (x) (= (length x) 2)) parameters)))
	 (sweep-parameters
	  (remove-if-not (lambda (x) (= (length x) 3)) parameters))
	 (vals (quasi-random-values (length sweep-parameters) count)))
    (iter:iter
      (iter:for v iter:in vals)
      (iter:collect
	  (apply
	   make-function
	   (append
	    fixed-parameters
	    (mapcan
	     (lambda (x seq)
	       (list (first x)
		     (val-from-range
		      (grid:aref v seq) (second x) (third x))))
	     sweep-parameters
	     (loop for i from 0 below (length sweep-parameters) collect i))))))))

(defun apply-to-arguments (function argument-order argument-list)
  "Apply the function to the arguments given, where the argument list is a list of (argument-name argument-value)."
  (apply function
	 (loop for arg in argument-order
	       collect (getf argument-list arg))))
