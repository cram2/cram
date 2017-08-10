;; Least squares estimation
;; Liam Healy 2013-03-29 14:16:21EDT least-squares.lisp
;; Time-stamp: <2015-11-22 11:48:42EST least-squares.lisp>

;; Copyright 2013, 2014 Liam M. Healy
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

;;; This requires GSLL

(in-package :antik)
(named-readtables:in-readtable :antik)

(export '(linear-least-squares-1d))

;;;;****************************************************************************
;;;; Linear least squares solver
;;;;****************************************************************************

;;; This should be consistent: now, 'observations is expected to be a list, 'times a grid
(defun linear-least-squares-1d (observations &optional (times #m(0.0 10.0 20.0 30.0 40.0)))
  "Find the one dimensional least squares (linear regression) solution of slope and intercept. Values returned: slope, intercept, and their standard errors."
  (metabang-bind:bind
      (((:values intercept slope intercept-variance covariance slope-variance sum-square-of-residuals)
	(gsl:linear-fit times (grid:make-simple-grid :grid-type 'grid:foreign-array :initial-contents observations)))
       (sample-variance (/ sum-square-of-residuals (- (grid:dim0 observations) 2))))
    (declare (ignore covariance))
    (values slope intercept
	    (sqrt (* sample-variance slope-variance))
	    (sqrt (* sample-variance intercept-variance)))))
