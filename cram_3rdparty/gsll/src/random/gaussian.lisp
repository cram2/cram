;; Gaussian distribution
;; Liam Healy, Sun Jul 16 2006 - 22:09
;; Time-stamp: <2010-01-17 10:19:46EST gaussian.lisp>
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

;;; /usr/include/gsl/gsl_randist.h
;;; /usr/include/gsl/gsl_cdf.h

(defmfun sample
    ((generator random-number-generator) (type (eql :gaussian))
     &key sigma)
  "gsl_ran_gaussian"
  (((mpointer generator) :pointer) (sigma :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A Gaussian random variate, with mean zero and
  standard deviation sigma.  The probability distribution for
  Gaussian random variates is
  p(x) dx = {1 \over \sqrt{2 \pi \sigma^2}} \exp (-x^2 / 2\sigma^2) dx
  for x in the range -\infty to +\infty.  Use the
  transformation z = \mu + x on the numbers returned by
  this function to obtain a Gaussian distribution with mean
  mu.  This function uses the Box-Mueller algorithm which requires two
  calls to the random number generator r.")

(defmfun gaussian-pdf (x sigma)
  "gsl_ran_gaussian_pdf" ((x :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "Compute the probability density p(x) at x
   for a Gaussian distribution with standard deviation sigma.")

(defmfun sample
    ((generator random-number-generator) (type (eql :gaussian-ziggurat))
     &key sigma)
  "gsl_ran_gaussian_ziggurat"
  (((mpointer generator) :pointer) (sigma :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "Compute a Gaussian random variate using the alternative
   Marsaglia-Tsang ziggurat method. The Ziggurat algorithm
   is the fastest available algorithm in most cases.")

(defmfun sample
    ((generator random-number-generator) (type (eql :gaussian-ratio-method))
     &key sigma)
  "gsl_ran_gaussian_ratio_method"
  (((mpointer generator) :pointer) (sigma :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "Compute a Gaussian random variate using the Kinderman-Monahan-Leva
   ratio method.")

(defmfun sample
    ((generator random-number-generator) (type (eql :ugaussian)) &key)
  "gsl_ran_ugaussian" (((mpointer generator) :pointer))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "Compute results for the unit Gaussian distribution,
   equivalent to the #'sample :gaussian with a standard deviation of
   one, sigma = 1.")

(defmfun ugaussian-pdf (x)
  "gsl_ran_ugaussian_pdf" ((x :double))
  :c-return :double
  :documentation			; FDL
  "Compute results for the unit Gaussian distribution,
   equivalent to the #'gaussian-pdf with a standard deviation of one,
   sigma = 1.")

(defmfun sample
    ((generator random-number-generator) (type (eql :ugaussian-ratio-method))
     &key)
  "gsl_ran_ugaussian_ratio_method"
  (((mpointer generator) :pointer))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "Compute results for the unit Gaussian distribution,
   equivalent to the #'sample :gaussian-ration-method with a standard
   deviation of one, sigma = 1.")

(defmfun gaussian-P (x sigma)
  "gsl_cdf_gaussian_P" ((x :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution function P(x) for the Gaussian
   distribution with standard deviation sigma.")

(defmfun gaussian-Q (x sigma)
  "gsl_cdf_gaussian_Q" ((x :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution function Q(x) for the Gaussian
   distribution with standard deviation sigma.")

(defmfun gaussian-Pinv (P sigma)
  "gsl_cdf_gaussian_Pinv" ((P :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution function P(x) for the Gaussian
   distribution with standard deviation sigma.")

(defmfun gaussian-Qinv (Q sigma)
  "gsl_cdf_gaussian_Qinv" ((Q :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution function Q(x) for the Gaussian
   distribution with standard deviation sigma.")

(defmfun ugaussian-P (x)
  "gsl_cdf_ugaussian_P" ((x :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution function P(x) for the Gaussian
   distribution with unit standard deviation.")

(defmfun ugaussian-Q (x)
  "gsl_cdf_ugaussian_Q" ((x :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution function Q(x) for the Gaussian
   distribution with unit standard deviation.")

(defmfun ugaussian-Pinv (P)
  "gsl_cdf_ugaussian_Pinv" ((P :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution function P(x) for the Gaussian
   distribution with unit standard deviation.")

(defmfun ugaussian-Qinv (Q)
  "gsl_cdf_ugaussian_Qinv" ((Q :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution function Q(x) for the Gaussian
   distribution with unit standard deviation.")

;;; Examples and unit test
(save-test
 gaussian
 (let ((rng (make-random-number-generator +mt19937+ 0)))
   (loop for i from 0 to 10
	 collect
	 (sample rng :gaussian :sigma 10.0d0)))
 (gaussian-pdf 0.0d0 10.0d0)
 (let ((rng (make-random-number-generator +mt19937+ 0)))
     (loop for i from 0 to 10
	   collect
	   (sample rng :gaussian-ziggurat :sigma 10.0d0)))
 ;; Given in examples in GSL documentation
 (ugaussian-p 2.0d0)
 (ugaussian-q 2.0d0)
 (ugaussian-pinv 0.9772498680518208d0)
 (ugaussian-qinv 0.02275013194817921d0))
