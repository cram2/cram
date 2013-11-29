;; Linear least squares, or linear regression
;; Liam Healy <2008-01-21 12:41:46EST linear-least-squares.lisp>
;; Time-stamp: <2010-06-30 19:57:28EDT linear-least-squares.lisp>
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

;;; /usr/include/gsl/gsl_fit.h
;;; /usr/include/gsl/gsl_multifit.h

;;;;****************************************************************************
;;;; Linear regression
;;;;****************************************************************************

;;; Error in GSL documentation or in code: vectors must be equal
;;; lengths but strides re not?  Shouldn't (floor length stride) be n?
;;; Error in GSL documentation for gsl_fit_linear_est, "c00" instead
;;; of "cov00" etc.  Last arg to gsl_fit_wmul is labelled sumsq but
;;; referred to as chisq.

(defmfun linear-fit
    (x y &optional weight (x-stride 1) (y-stride 1) (weight-stride 1))
  ("gsl_fit_linear" "gsl_fit_wlinear")
  ((((foreign-pointer x) :pointer) (x-stride sizet)
    ((foreign-pointer y) :pointer) (y-stride sizet)
    ((dim0 x) sizet)
    (c0 (:pointer :double)) (c1 (:pointer :double))
    (cov00 (:pointer :double)) (cov01 (:pointer :double))
    (cov11 (:pointer :double)) (sumsq (:pointer :double)))
   (((foreign-pointer x) :pointer) (x-stride sizet)
    ((foreign-pointer weight) :pointer) (weight-stride sizet)
    ((foreign-pointer y) :pointer) (y-stride sizet)
    ((dim0 x) sizet)
    (c0 (:pointer :double)) (c1 (:pointer :double))
    (cov00 (:pointer :double)) (cov01 (:pointer :double))
    (cov11 (:pointer :double)) (sumsq (:pointer :double))))
  :inputs (x weight y)
  :switch (weight weight-stride)
  :documentation			; FDL
  "Compute the best-fit linear regression coefficients
   c0, c1 of the model Y = c_0 + c_1 X for the weighted or unweighted
   dataset (x, y), two vectors of equal length with strides
   x-stride and y-stride, and return as the first two values.
   The vector weight if given, of the same length
   and stride w-stride, specifies the weight of each datapoint. The
   weight is the reciprocal of the variance for each datapoint in y.

   The covariance matrix for the parameters (c0, c1) is
   computed using the weights and returned via the parameters
   (cov00, cov01, c0v01) as the next three values.  The weighted or
   unweighted sum of squares of the residuals from the best-fit line,
   \chi^2, is returned in as the last value.")

(defmfun linear-estimate (x c0 c1 cov00 cov01 cov11)
  "gsl_fit_linear_est"
  ((x :double) (c0 :double) (c1 :double)
   (cov00 :double) (cov01 :double) (cov11 :double)
   (y (:pointer :double)) (y-error (:pointer :double)))
  :documentation			; FDL
  "Use the best-fit linear regression coefficients
   c0, c1 and their covariance
   cov00, cov01, cov11 to compute the fitted function
   y and its standard deviation y-error for the model
   Y = c_0 + c_1 X at the point x.")

;;;;****************************************************************************
;;;; Linear fitting without a constant term
;;;;****************************************************************************

(defmfun multiplier-fit
    (x y &optional weight (x-stride 1) (y-stride 1) (weight-stride 1))
  ("gsl_fit_mul" "gsl_fit_wmul")
  ((((foreign-pointer x) :pointer) (x-stride sizet)
    ((foreign-pointer y) :pointer) (y-stride sizet)
    ((dim0 x) sizet)
    (c1 (:pointer :double)) (cov11 (:pointer :double))
    (sumsq (:pointer :double)))
   (((foreign-pointer x) :pointer) (x-stride sizet)
    ((foreign-pointer weight) :pointer) (weight-stride sizet)
    ((foreign-pointer y) :pointer) (y-stride sizet)
    ((dim0 x) sizet)
    (c1 (:pointer :double)) (cov11 (:pointer :double))
    (sumsq (:pointer :double))))
  :inputs (x y weight)
  :switch (weight weight-stride)
  :documentation			; FDL
  "Compute the best-fit linear regression coefficient
   c1 of the model Y = c_1 X for the weighted or unweighted datasets
   (x, y), two vectors of equal length with strides
   x-stride and y-stride.  The vector weight of the same length
   and of stride w-stide specifies the weight of each datapoint. The
   weight is the reciprocal of the variance for each datapoint in y.
 
   The variance of the parameter c1 is computed using the weights
   and returned as the second value.  The weighted sum of
   squares of the residuals from the best-fit line, \chi^2, is
   returned as the last value.")

(defmfun multiplier-estimate (x c1 cov11)
  "gsl_fit_mul_est"
  ((x :double) (c1 :double) (cov11 :double)
   (y (:pointer :double)) (y-error (:pointer :double)))
  :documentation			; FDL
  "Use the best-fit linear regression coefficient
   c1 and its covariance cov11 to compute the fitted function
   y and its standard deviation y-error for the model
   Y = c_0 + c_1 X at the point x.")

;;;;****************************************************************************
;;;; Multiparameter fitting
;;;;****************************************************************************

(defmobject fit-workspace
    "gsl_multifit_linear"
  ((number-of-observations sizet) (number-of-parameters sizet))
  "multi-dimensional root solver with function only"
  :documentation
  "Make a workspace for a multidimensional linear least-squares fit.")

(defun size-array (array-or-size)
  (if (numberp array-or-size)
      array-or-size
      (dim0 array-or-size)))

(defun default-covariance (parameters-or-size)
  (grid:make-foreign-array
   'double-float
   :dimensions
   (let ((s (size-array parameters-or-size))) (list s s))))

(defun default-lls-workspace (observations parameters-or-size)
  (make-fit-workspace
   (dim0 observations) (size-array parameters-or-size)))

(export 'linear-mfit)
(defun linear-mfit
    (model observations parameters-or-size
     &optional weight tolerance
     (covariance (default-covariance parameters-or-size))
     (workspace (default-lls-workspace observations parameters-or-size)))
  "Compute the best-fit parameters c of the weighted or unweighted
   model y = X c for the observations y and optional weights
   and the model matrix X.  The covariance matrix of
   the model parameters is computed with the given weights.  The
   weighted sum of squares of the residuals from the best-fit,
   chi^2, is returned as the last value.

   The best-fit is found by singular value decomposition of the matrix
   model using the preallocated workspace provided. The modified
   Golub-Reinsch SVD algorithm is used for the unweighted solution,
   with column scaling to improve the accuracy of the singular values.
   Any components which have zero singular value (to machine
   precision) are discarded from the fit.

   If tolerance is a double-float, the SVD algorithm is used.
   If it is nil the non-svd algorithm is used."
  (if tolerance
      (linear-mfit-svd
       model observations parameters-or-size
       tolerance weight covariance workspace)
      (linear-mfit-nosvd
       model observations parameters-or-size
       weight covariance workspace)))

(defmfun linear-mfit-nosvd
    (model observations parameters-or-size
	   &optional
	   weight
	   (covariance (default-covariance parameters-or-size))
	   (workspace (default-lls-workspace observations parameters-or-size))
	   &aux
	   (parameters (vdf parameters-or-size)))
  ("gsl_multifit_linear" "gsl_multifit_wlinear")
  ((((mpointer model) :pointer)
    ((mpointer observations) :pointer)
    ((mpointer parameters) :pointer)
    ((mpointer covariance) :pointer)
    (chisq (:pointer :double))
    ((mpointer workspace) :pointer))
   (((mpointer model) :pointer)
    ((mpointer weight) :pointer)
    ((mpointer observations) :pointer)
    ((mpointer parameters) :pointer)
    ((mpointer covariance) :pointer)
    (chisq (:pointer :double))
    ((mpointer workspace) :pointer)))
  :inputs (model weight observations)
  :outputs (parameters covariance)
  :switch (weight)
  :return (parameters covariance (grid:dcref chisq))
  :export nil
  :index linear-mfit
  :documentation			; FDL
  "Compute the best-fit parameters c of the weighted or unweighted
   model y = X c for the observations y and optional weights
   and the model matrix X.  The covariance matrix of
   the model parameters is computed with the given weights.  The
   weighted sum of squares of the residuals from the best-fit,
   chi^2, is returned as the last value.

   The best-fit is found by singular value decomposition of the matrix
   model using the preallocated workspace provided. The modified
   Golub-Reinsch SVD algorithm is used for the unweighted solution,
   with column scaling to improve the accuracy of the singular values.
   Any components which have zero singular value (to machine
   precision) are discarded from the fit.")

(defmfun linear-mfit-svd
    (model observations parameters-or-size tolerance
	   &optional weight
	   (covariance (default-covariance parameters-or-size))
	   (workspace (default-lls-workspace observations parameters-or-size))
	   &aux (parameters (vdf parameters-or-size)))
  ("gsl_multifit_linear_svd" "gsl_multifit_wlinear_svd")
  ((((mpointer model) :pointer)
    ((mpointer observations) :pointer)
    (tolerance :double)
    (rank (:pointer sizet))
    ((mpointer parameters) :pointer)
    ((mpointer covariance) :pointer)
    (chisq (:pointer :double))
    ((mpointer workspace) :pointer))
   (((mpointer model) :pointer)
    ((mpointer weight) :pointer)
    ((mpointer observations) :pointer)
    (tolerance :double)
    (rank (:pointer sizet))
    ((mpointer parameters) :pointer)
    ((mpointer covariance) :pointer)
    (chisq (:pointer :double))
    ((mpointer workspace) :pointer)))
  :inputs (model weight observations)
  :outputs (parameters covariance)
  :switch (weight)
  :return ((grid:dcref chisq) (scref rank))
  :export nil
  :index linear-mfit
  :documentation			; FDL
  "Compute the best-fit parameters c of the weighted or unweighted
   model y = X c for the observations y and weights and the model
   matrix X.  The covariance matrix of the model parameters is
   computed with the given weights.  The weighted or unweighted sum of
   squares of the residuals from the best-fit, chi^2, is returned as
   the first value.

   The best-fit is found by singular value decomposition of the matrix
   model using the preallocated workspace provided.  The modified
   Golub-Reinsch SVD algorithm is used, with column scaling to improve
   the accuracy of the singular values (unweighted).  Any components
   which have zero singular value (to machine precision) are discarded
   from the fit.  In the second form of the function the components
   are discarded if the ratio of singular values s_i/s_0 falls below
   the user-specified tolerance, and the effective rank is returned as
   the second value.")

(defmfun multi-linear-estimate (x coefficients covariance)
  "gsl_multifit_linear_est"
  (((mpointer x) :pointer) ((mpointer coefficients) :pointer)
   ((mpointer covariance) :pointer)
   (y (:pointer :double)) (y-error (:pointer :double)))
  :inputs (x coefficients covariance)
  :documentation			; FDL
  "Use the best-fit multilinear regression coefficients
   and their covariance matrix to compute the fitted function value
   y and its standard deviation for the model y = x.c
   at the point x.")

(defmfun multi-linear-residuals
    (x observations coefficients
       &optional
       (residuals
	(grid:make-foreign-array 'double-float :dimensions (dimensions observations))))
  "gsl_multifit_linear_residuals"
  (((mpointer x) :pointer) ((mpointer observations) :pointer)
   ((mpointer coefficients) :pointer) ((mpointer residuals) :pointer))
  :inputs (x observations coefficients)
  :outputs (residuals)
  :gsl-version (1 11)
  :documentation			; FDL
  "Compute the vector of residuals r = y - X c for the observations y,
  coefficients c and matrix of predictor variables X.")

;;;;****************************************************************************
;;;; Examples
;;;;****************************************************************************

(defun linear-least-squares-univariate-example (&optional (print-steps t))
  "First example in Section 36.5 of the GSL manual."
  ;; Results not given in manual so not verified yet.
  (let ((x #m(1970.0d0 1980.0d0 1990.0d0 2000.0d0))
	(y #m(12.0d0 11.0d0 14.0d0 13.0d0))
	(w #m(0.1d0 0.2d0 0.3d0 0.4d0)))
    (multiple-value-bind (c0 c1 cov00 cov01 cov11 chisq)
	(linear-fit x y w)
      (when print-steps
	(format t "Best fit: Y = ~8,5f + ~8,5f X~&" c0 c1)
	(format t "Covariance matrix:~&[~12,5f ~12,5f~&~12,5f ~12,5f]~&"
		cov00 cov01 cov01 cov11)
	(format t "Chisq = ~g~&" chisq)
	(loop for i from 0 below (dim0 x)
	   do
	   (format t "data: ~12,5f ~12,5f ~12,5f~&"
		   (grid:gref x i)
		   (grid:gref y i)
		   (/ (grid:gref w i))))
	(loop for i from -30 below 130 by 10 ; don't print everything
	   for
	   xf = (+ (grid:gref x 0)
		   (* (/ i 100)
		      (- (grid:gref x (1- (dim0 x)))
			 (grid:gref x 0))))
	   do
	   (multiple-value-bind (yf yferr)
	       (linear-estimate xf c0 c1 cov00 cov01 cov11)
	     (format t "fit:~6t~g ~g~&" xf yf)
	     (format t "high:~6t~g ~g~&" xf (+ yf yferr))
	     (format t "low:~6t~g ~g~&" xf (- yf yferr)))))
      (values c0 c1 cov00 cov01 cov11 chisq))))

(defun mv-linear-least-squares-data ()
  "Generate data for second example in Section 36.5 of the GSL
   manual."
  (let ((rng (make-random-number-generator +mt19937+ 0)))
    (loop for x from 1/10 below 2 by 1/10
	  for xd = (coerce x 'double-float)
	  for y0 = (exp xd)
	  for sigma = (* 0.1d0 y0)
	  collect
	  (list xd (+ y0 (sample rng :gaussian :sigma sigma)) sigma))))

(defun linear-least-squares-multivariate-example (data &optional (print-details t))
  "Second example in Section 36.5 of the GSL manual.  Returns the
   coefficients of x^0, x^1, x^2 for the best fit, and the chi
   squared."
  (let* ((n (length data))
	 (x (grid:make-foreign-array 'double-float :dimensions (list n 3)))
	 (y (grid:make-foreign-array 'double-float :dimensions n))
	 (w (grid:make-foreign-array 'double-float :dimensions n)))
    (loop for i from 0
       for row in data do
       (setf (grid:gref X i 0) 1.0d0
	     (grid:gref X i 1) (first row)
	     (grid:gref X i 2) (expt (first row) 2)
	     (grid:gref y i) (second row)
	     (grid:gref w i) (/ (expt (third row) 2))))
    (multiple-value-bind (parameters cov chisq)
	(linear-mfit X y 3 w)
      (when print-details
	(format t "Best fit: Y = ~10,8f + ~10,8f X + ~10,8f X^2~&"
		(grid:gref parameters 0) (grid:gref parameters 1) (grid:gref parameters 2))
	(format t "Covariance matrix:~&")
	(format t "~10,8f ~10,8f ~10,8f~&"
		(grid:gref cov 0 0) (grid:gref cov 0 1) (grid:gref cov 0 2))
	(format t "~10,8f ~10,8f ~10,8f~&"
		(grid:gref cov 1 0) (grid:gref cov 1 1) (grid:gref cov 1 2))
	(format t "~10,8f ~10,8f ~10,8f~&"
		(grid:gref cov 2 0) (grid:gref cov 2 1) (grid:gref cov 2 2))
	(format t "Chisq = ~10,6f~&" chisq))
      (values
       (grid:gref parameters 0) (grid:gref parameters 1) (grid:gref parameters 2)
       chisq))))

(save-test linear-least-squares
 (linear-least-squares-univariate-example nil)
 (linear-least-squares-multivariate-example (mv-linear-least-squares-data) nil))
