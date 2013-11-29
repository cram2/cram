;; Mean, standard deviation, and variance    
;; Liam Healy, Sat Dec  2 2006 - 22:15
;; Time-stamp: <2010-06-30 19:57:28EDT mean-variance.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009 Liam M. Healy
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

;;; To do: stride other than 1 when that information is available from
;;; the vector.

;;;;****************************************************************************
;;;; Mean and weighted mean
;;;;****************************************************************************

(defmfun mean ((array both))
  ("gsl_stats" :type "_mean")
  (((foreign-pointer array) :pointer) (1 :int) ((size array) sizet))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (array)
  :documentation			; FDL
   "The arithmetic mean of the array.
   The arithmetic mean, or sample mean, is denoted by
   \Hat\mu and defined as \Hat\mu = (1/N) \sum x_i.  Returns a double-float.")

(defmfun weighted-mean ((array both) (weights both))
  ("gsl_stats" :type "_wmean")
  (((foreign-pointer weights) :pointer) (1 :int)
   ((foreign-pointer array) :pointer) (1 :int)
   ((size array) sizet))
  :definition :generic
  :element-types :float
  :c-return :element-c-type
  :inputs (array weights)
  :documentation			; FDL
  "The weighted mean of the dataset, using the set of weights
    The weighted mean is defined as
    \Hat\mu = (\sum w_i x_i) / (\sum w_i).")

;;;;****************************************************************************
;;;; Variance
;;;;****************************************************************************

(defmfun variance ((array both) &optional mean)
  (("gsl_stats" :type "_variance")
   ("gsl_stats" :type "_variance_m"))
  ((((foreign-pointer array) :pointer) (1 :int) ((size array) sizet))
   (((foreign-pointer array) :pointer) (1 :int)
   ((size array) sizet) (mean :double)))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (array)
  :documentation			; FDL
  "The estimated, or sample, variance of data.  The
   estimated variance is denoted by \Hat\sigma^2 and is defined by
   \Hat\sigma^2 = (1/(N-1)) \sum (x_i - \Hat\mu)^2
   where x_i are the elements of the dataset data.  Note that
   the normalization factor of 1/(N-1) results from the derivation
   of \Hat\sigma^2 as an unbiased estimator of the population
   variance \sigma^2.  For samples drawn from a gaussian distribution
   the variance of \Hat\sigma^2 itself is 2 \sigma^4 / N.
   If the mean value is known, it may be supplied which will use more
   efficient routines to compute the variance.")

(defmfun weighted-variance ((array both) (weights both) &optional mean)
  (("gsl_stats" :type "_wvariance")
   ("gsl_stats" :type "_wvariance_m"))
  ((((foreign-pointer weights) :pointer) (1 :int)
    ((foreign-pointer array) :pointer) (1 :int)
    ((size array) sizet))
   (((foreign-pointer weights) :pointer) (1 :int)
    ((foreign-pointer array) :pointer) (1 :int)
    ((size array) sizet) (mean :double)))
  :definition :generic
  :element-types :float
  :c-return :double
  :inputs (array weights)
  :documentation			; FDL
  "The estimated variance of a weighted dataset is defined as
   \Hat\sigma^2 = ((\sum w_i)/((\sum w_i)^2 - \sum (w_i^2))) 
                \sum w_i (x_i - \Hat\mu)^2
   Note that this expression reduces to an unweighted variance with the
   familiar 1/(N-1) factor when there are N equal non-zero weights.
   If the mean value is known, it may be supplied which will use more
   efficient routines to compute the variance.")

;;;;****************************************************************************
;;;; Standard deviation
;;;;****************************************************************************

(defmfun standard-deviation ((array both) &optional mean)
  (("gsl_stats" :type "_sd")
   ("gsl_stats" :type "_sd_m"))
  ((((foreign-pointer array) :pointer) (1 :int) ((size array) sizet))
   (((foreign-pointer array) :pointer) (1 :int)
   ((size array) sizet) (mean :double)))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (array)
  :documentation			; FDL
  "The standard deviation, square root of the variance.
   If the mean value is known, it may be supplied which will use more
   efficient routines to compute the variance.")

(defmfun weighted-standard-deviation ((array both) (weights both) &optional mean)
  (("gsl_stats" :type "_wsd")
   ("gsl_stats" :type "_wsd_m"))
  ((((foreign-pointer weights) :pointer) (1 :int)
    ((foreign-pointer array) :pointer) (1 :int)
    ((size array) sizet))
   (((foreign-pointer weights) :pointer) (1 :int)
    ((foreign-pointer array) :pointer) (1 :int)
    ((size array) sizet) (mean :double)))
  :definition :generic
  :element-types :float
  :c-return :double
  :inputs (array weights)
  :documentation			; FDL
  "The weighted standard deviation, square root of the variance.
   If the mean value is known, it may be supplied which will use more
   efficient routines to compute the variance.")

;;;;****************************************************************************
;;;; With fixed mean
;;;;****************************************************************************

(defmfun variance-with-fixed-mean ((array both) mean)
  ("gsl_stats" :type "_variance_with_fixed_mean")
  (((foreign-pointer array) :pointer) (1 :int)
   ((size array) sizet) (mean :double))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (array)
  :documentation			; FDL
  "An unbiased estimate of the variance of
    data when the population mean of the underlying
    distribution is known a priori.  In this case the estimator for
    the variance uses the factor 1/N and the sample mean
    \Hat\mu is replaced by the known population mean \mu,
    \Hat\sigma^2 = (1/N) \sum (x_i - \mu)^2.")

(defmfun standard-deviation-with-fixed-mean ((array both) mean)
  ("gsl_stats" :type "_sd_with_fixed_mean")
  (((foreign-pointer array) :pointer) (1 :int)
   ((size array) sizet) (mean :double))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (array)
  :documentation			; FDL
   "The standard deviation of data for a fixed population
    mean.  The result is the square root of the
    corresponding variance function.")

;;;;****************************************************************************
;;;; Weighted with fixed mean
;;;;****************************************************************************

(defmfun weighted-variance-with-fixed-mean
    ((array both) (weights both) mean)
  ("gsl_stats" :type "_wvariance_with_fixed_mean")
  (((foreign-pointer weights) :pointer) (1 :int)
   ((foreign-pointer array) :pointer) (1 :int)
   ((size array) sizet) (mean :double))
  :definition :generic
  :element-types :float
  :c-return :double
  :inputs (array weights)
  :documentation			; FDL
  "An unbiased estimate of the variance of weighted
    dataset when the population mean of the underlying
    distribution is known a priori.  In this case the estimator for
    the variance replaces the sample mean \Hat\mu by the known
    population mean \mu,
    \Hat\sigma^2 = (\sum w_i (x_i - \mu)^2) / (\sum w_i).")

(defmfun weighted-standard-deviation-with-fixed-mean
    ((array both) (weights both) mean)
  ("gsl_stats" :type "_wsd_with_fixed_mean")
  (((foreign-pointer weights) :pointer) (1 :int)
   ((foreign-pointer array) :pointer) (1 :int)
   ((size array) sizet) (mean :double))
  :definition :generic
  :element-types :float
  :c-return :double
  :inputs (array weights)
  :documentation			; FDL
  "The square root of the corresponding variance
   function #'weighted-variance-with-fixed-mean.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(generate-all-array-tests vector-mean :no-complex
 (let ((v1 (array-default 8)))
   (mean v1)))

(generate-all-array-tests matrix-mean :no-complex
 (let ((m1 (array-default '(3 3))))
   (mean m1)))

(generate-all-array-tests vector-variance :no-complex
 (let ((v1 (array-default 8)))
   (variance v1)))

(generate-all-array-tests matrix-variance :no-complex
 (let ((m1 (array-default '(3 3))))
   (variance m1)))

(generate-all-array-tests vector-variance-with-mean :no-complex
 (let ((v1 (array-default 8)))
   (variance v1 (mean v1))))

(generate-all-array-tests matrix-variance-with-mean :no-complex
 (let ((m1 (array-default '(3 3))))
   (variance m1 (mean m1))))

(generate-all-array-tests vector-standard-deviation :no-complex
 (let ((v1 (array-default 8)))
   (standard-deviation v1)))

(generate-all-array-tests matrix-standard-deviation :no-complex
 (let ((m1 (array-default '(3 3))))
   (standard-deviation m1)))

(generate-all-array-tests vector-standard-deviation-with-mean :no-complex
 (let ((v1 (array-default 8)))
   (standard-deviation v1 (mean v1))))

(generate-all-array-tests matrix-standard-deviation-with-mean :no-complex
 (let ((m1 (array-default '(3 3))))
   (standard-deviation m1 (mean m1))))

(generate-all-array-tests vector-variance-with-fixed-mean :no-complex
 (let ((v1 (array-default 8)))
   (variance-with-fixed-mean v1 (mean v1))))

(generate-all-array-tests matrix-variance-with-fixed-mean :no-complex
 (let ((m1 (array-default '(3 3))))
   (variance-with-fixed-mean m1 (mean m1))))

(generate-all-array-tests vector-standard-deviation-with-fixed-mean :no-complex
 (let ((v1 (array-default 8)))
   (standard-deviation-with-fixed-mean v1 (mean v1))))

(generate-all-array-tests matrix-standard-deviation-with-fixed-mean :no-complex
 (let ((m1 (array-default '(3 3))))
   (standard-deviation-with-fixed-mean m1 (mean m1))))

#|
;;; Weighted mean seems to be in error in GSL
;;; Hold off on all weghted tests until this is resolved.
(generate-all-array-tests weighted-mean :float
 (let ((v1 (array-default 8))
	(v2 (array-default 8)))
   (loop for i below (first (dimensions v2))
      do (setf (grid:gref v2 i) (abs (grid:gref v2 i))))
   (weighted-mean v1 v2)))
|#

