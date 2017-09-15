;; Use this for speed tests
;; Liam Healy 2010-11-14 18:45:53EST timing.lisp
;; Time-stamp: <2011-05-23 22:50:57EDT timing.lisp>
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

(in-package :cl-user)

(defun foreign-array-test (dim)
  "Test for foreign-array speed, adapted from Sebastian Sturm's code."
  (let ((input (grid:make-foreign-array
		'double-float
		:dimensions dim
		:initial-element 1.0d0))
	(output (grid:make-foreign-array 'double-float :dimensions dim)))
    (declare (type grid:vector-double-float input output))
    (let ((tv0 0.0d0) (tv1 0.0d0))
      (declare (type double-float tv0 tv1))
      (iter (for i from 0 below dim)
	    (setf tv0 0.0d0)
	    (iter (for m from 0 to i)
		  (iter (for n from i below dim)
			(setf tv1 0.0d0)
			(iter (for k from m to n)
			      (incf tv1 (grid:aref input k)))
			(incf tv0 (expt tv1 -2))))
	    (grid:gsetf (grid:aref* output i) (- (grid:aref input i) tv0))))))
