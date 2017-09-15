;; Example spline
;; Liam Healy, Sat Nov 10 2007 - 21:18
;; Time-stamp: <2011-05-26 12:37:34EDT spline-example.lisp>
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

(defun spline-example (&optional (step 0.01d0))
  "The first example in Sec. 26.7 of the GSL manual."
  (let* ((xarr
	  (grid:make-foreign-array
	   'double-float
	   :initial-contents
	   (loop for i from 0.0d0 below 10.0d0
	      collect (+ i (* 0.5d0 (sin i))))))
	 (yarr
	  (grid:make-foreign-array
	   'double-float
	   :initial-contents
	   (loop for i from 0.0d0 below 10.0d0
	      collect (+ i (cos (expt i 2))))))
	 (spline (make-spline +cubic-spline-interpolation+ xarr yarr)))
    (loop for xi from (grid:aref xarr 0) below (grid:aref xarr 9) by step
       collect (list xi (evaluate spline xi)))))

(defun evaluate-integral-example (&optional (intervals 4))
  "Evaluate integral of sin(x) in interval 0-pi.  sin(x) is tabulated
   over a 0-2pi interval and interpolated with
   +periodic-cubic-spline-interpolation+"
  (let* ((xarr 
	  (loop with step = (/ (* 2.0 dpi) intervals)
	     for i from 0 upto intervals
	     collect (* i step)))
	 (xmarr (grid:make-foreign-array 'double-float :initial-contents xarr))
	 (ymarr
	  (grid:make-foreign-array 'double-float :initial-contents (mapcar 'sin xarr)))
	 (spline
	  (make-spline +periodic-cubic-spline-interpolation+ xmarr ymarr)))
    (evaluate-integral spline 0d0 dpi)))

(save-test interpolation
 (spline-example 0.1d0)
 (evaluate-integral-example 100))
