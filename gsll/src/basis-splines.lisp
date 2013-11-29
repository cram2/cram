;; Basis splines.
;; Liam Healy 2008-02-18 14:43:20EST basis-splines.lisp
;; Time-stamp: <2010-06-30 19:57:28EDT basis-splines.lisp>
;;
;; Copyright 2008, 2009 Liam M. Healy
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

;;; /usr/include/gsl/gsl_bspline.h

;;; Should be subclass of interpolation?

(defmobject basis-spline "gsl_bspline"
  ((order sizet) (number-of-breakpoints sizet))
  "basis spline"
  :gsl-version (1 9)
  :documentation			; FDL
  "Allocate a workspace for computing B-splines. The number of
   breakpoints is given by number-of-breakpoints.  This leads to n =
   nbreak + k - 2 basis functions where k = order. Cubic B-splines are
   specified by k = 4. The size of the workspace is O(5k + nbreak).")

;;; Constructing the knots vector
(defmfun knots (breakpoints workspace)
  "gsl_bspline_knots"
  (((mpointer breakpoints) :pointer) ((mpointer workspace) :pointer))
  :gsl-version (1 9)
  :documentation			; FDL
  "Compute the knots associated with the given breakpoints and store
   them in the workspace.")

(defmfun uniform-knots (a b workspace)
  "gsl_bspline_knots_uniform"
  ((a :double) (b :double) ((mpointer workspace) :pointer))
  :return (workspace)
  :gsl-version (1 9)
  :documentation			; FDL
  "Compute knots uniformly on the interval [a, b] and store
   them in the workspace.")

;;; Evaluation of B-splines

(defmfun evaluate
    ((workspace basis-spline) x
     &key (B (grid:make-foreign-array 'double-float
			  :dimensions (number-of-coefficients workspace))))
  "gsl_bspline_eval"
  ((x :double) ((mpointer B) :pointer) ((mpointer workspace) :pointer))
  :definition :method
  :outputs (B)
  :gsl-version (1 9)
  :documentation			; FDL
  "Evaluate all B-spline basis functions at the position x and store
   them in the GSL vector B, so that the ith element of B is B_i(x).
   B must be of length n = nbreak + k - 2. This value is
   also stored in the workspace. It is far more
   efficient to compute all of the basis functions at once than to
   compute them individually, due to the nature of the defining
   recurrence relation.")

;;; Query settings of the B-spline
;;; These are not documented but are in
;;; /usr/include/gsl/gsl_bspline.h; are they officially supported?

(defmfun number-of-coefficients (bspline)
  "gsl_bspline_ncoeffs"
  (((mpointer bspline) :pointer))
  :c-return sizet
  :gsl-version (1 9))

(defmfun order ((bspline basis-spline))
  "gsl_bspline_order"
  (((mpointer bspline) :pointer))
  :definition :method
  :c-return sizet
  :gsl-version (1 9))

(defmfun number-of-breakpoints (bspline)
  "gsl_bspline_nbreak"
  (((mpointer bspline) :pointer))
  :c-return sizet
  :gsl-version (1 9)
  :documentation "The number of breakpoints of the basis spline bspline.")

(defmfun breakpoint (i bspline)
  "gsl_bspline_breakpoint"
  ((i sizet) ((mpointer bspline) :pointer))
  :c-return :double
  :gsl-version (1 9)
  :documentation "The ith breakpoint of the basis spline bspline.")

;;; Examples and unit test

(defun bspline-example (&optional (ncoeffs 8))
  (let* ((order 4)			; cubic
	 (ndata 200)
	 (nbreak (+ ncoeffs 2 (- order)))
	 (bw (make-basis-spline order nbreak))
	 (mw (make-fit-workspace ndata ncoeffs))
	 (rng (make-random-number-generator +mt19937+ 0))
	 (B (grid:make-foreign-array 'double-float :dimensions ncoeffs))
	 (c (grid:make-foreign-array 'double-float :dimensions ncoeffs))
	 (cov (grid:make-foreign-array 'double-float :dimensions (list ncoeffs ncoeffs)))
	 (w (grid:make-foreign-array 'double-float :dimensions ndata))
	 (x (grid:make-foreign-array 'double-float :dimensions ndata))
	 (y (grid:make-foreign-array 'double-float :dimensions ndata))
	 (Xmatrix (grid:make-foreign-array 'double-float :dimensions (list ndata ncoeffs)))
	 (sigma 0.1d0))
    ;; The data to be fitted.
    (dotimes (i ndata)
      (let* ((xi (coerce (* i (/ 15 (1- ndata))) 'double-float))
	     (yi (+ (* (cos xi) (exp (* -0.1d0 xi)))
		    (sample rng :gaussian :sigma sigma))))
	(setf (grid:gref x i) xi
	      (grid:gref y i) yi
	      (grid:gref w i) (/ (expt sigma 2)))))
    ;; Uniform breakpoints [0, 15]
    (uniform-knots 0.0d0 15.0d0 bw)
    ;; Fit matrix
    (dotimes (i ndata)
      ;; Compute B_j
      (evaluate bw (grid:gref x i) :b B)
      ;; Fill in row i of X
      (dotimes (j ncoeffs)
	(setf (grid:gref Xmatrix i j) (grid:gref B j))))
    ;; Do the fit
    (linear-mfit Xmatrix y c w nil cov mw)
    ;; Return the smoothed curve
    (loop for xi from 0.0d0 to 15.0d0 by 0.1d0
       with yval and yerr
       do
       (evaluate bw xi :b B)
       (multiple-value-setq (yval yerr)
	 (multi-linear-estimate B c cov))
       collect yval into yvals 
       collect yerr into yerrs
       finally (return (values yvals yerrs)))))

(save-test basis-spline (bspline-example))
