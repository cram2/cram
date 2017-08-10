;; Minimize or maximize the function in more than one variable.
;; Liam Healy 2011-09-09 14:59:46EDT multi-dim.lisp
;; Time-stamp: <2011-09-09 15:53:42EDT multi-dim.lisp>

;; Copyright 2011 Liam M. Healy
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

;;; This requires GSLL

(defun minimize-multidimensional-noderivative
    (function starting-point &optional (method gsl:+simplex-nelder-mead-on2+))
  (let* ((initial
	  (if (typep starting-point 'grid:foreign-array)
	      starting-point
	      (grid:make-foreign-array 'double-float :initial-contents starting-point)))
	 (ndim (first (grid:dimensions initial)))
	 (step-size (grid:make-foreign-array 'double-float :dimensions ndim :initial-element 1.0d0))
	 (minimizer
	  (gsl:make-multi-dimensional-minimizer-f method ndim function initial step-size)))
    (loop with status = T and size
	  for iter from 0 below 100
	  while status
	  do (gsl:iterate minimizer)
	     (setf size
		   (gsl:size minimizer)
		   status
		   (not (gsl:min-test-size size 1.0d-2)))
	  finally
       (return
	 (let ((x (gsl:solution minimizer)))
	   (values (grid:aref x 0) (grid:aref x 1) (gsl:function-value minimizer)))))))

;;; Example 
;;; (minimize-multidimensional-noderivative 'gsl::paraboloid-scalar '(5.0d0 7.0d0))
;;; same as (in-package :gsl)
;;; (multimin-example-no-derivative +simplex-nelder-mead-on2+ nil)
