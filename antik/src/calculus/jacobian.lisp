;; Find the Jacobian matrix of a multivariate function
;; Liam Healy 2011-10-15 12:40:05EDT jacobian.lisp
;; Time-stamp: <2015-11-21 15:06:38EST jacobian.lisp>

;; Copyright 2011, 2013, 2015 Liam M. Healy
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

;;; This does not use GSLL's central-difference, because that is scalar only and it would be inefficient to repeatedly re-evaluate the function to get the various components.

(in-package :antik)
(export 'jacobian-matrix)

(defun jacobian-matrix
    (function point step
     &optional (matrix-or-nrows (grid:dim0 (funcall function point))))
  "The matrix of partial derivatives of the function.  If the number of rows nrows is not supplied, it is determined by evaluating the function at the point; supplying this information avoids that calculation.  Step must be a scalar."
  ;; This cannot currently use pq.
  (let ((matrix (if (grid:gridp matrix-or-nrows)
		    matrix-or-nrows
		    (grid:make-simple-grid
		     :dimensions (list (grid:dim0 point) matrix-or-nrows)
		     :grid-type (grid:gridp point)))))
    (iter (iter:for col from 0 below (grid:dim1 matrix))
      (let ((pos (grid:copy point :grid-type grid:*default-grid-type*))
	    (neg (grid:copy point :grid-type grid:*default-grid-type*)))
	(incf (grid:aref pos col) (/ step 2))
	(decf (grid:aref neg col) (/ step 2))
	(iter:for column = (/ (- (funcall function pos) (funcall function neg)) step))
	(unless matrix
	  (setf matrix
		(grid:make-simple-grid
		 :dimensions (list (grid:dim0 point) (grid:dim0 column))
		 :grid-type (grid:gridp point))))
	(setf (grid:column matrix col)
	      )))
    matrix))
