;; Lognormal distribution
;; Liam Healy, Sat Sep 30 2006
;; Time-stamp: <2010-01-17 10:27:47EST lognormal.lisp>
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

(defmfun sample
    ((generator random-number-generator) (type (eql :lognormal))
     &key zeta sigma)
  "gsl_ran_lognormal"
  (((mpointer generator) :pointer) (zeta :double) (sigma :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the lognormal distribution.
   The distribution function is
   p(x) dx = {1 \over x \sqrt{2 \pi \sigma^2}} \exp(-(\ln(x) - \zeta)^2/2 \sigma^2) dx
   for x > 0.")

(defmfun lognormal-pdf (x zeta sigma)
  "gsl_ran_lognormal_pdf" ((x :double) (zeta :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at X
   for a lognormal distribution with parameters zeta and sigma,
   using the formula given in #'sample :lognormal.")

(defmfun lognormal-P (x zeta sigma)
  "gsl_cdf_lognormal_P" ((x :double) (zeta :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(x) for the lognormal distribution with parameters zeta and sigma.")

(defmfun lognormal-Q (x zeta sigma)
  "gsl_cdf_lognormal_Q" ((x :double) (zeta :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  Q(x) for the lognormal distribution with parameters
  zeta and sigma.")

(defmfun lognormal-Pinv (P zeta sigma)
  "gsl_cdf_lognormal_Pinv" ((P :double) (zeta :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
  P(x) for the lognormal distribution with parameters
  zeta and sigma.")

(defmfun lognormal-Qinv (Q zeta sigma)
  "gsl_cdf_lognormal_Qinv" ((Q :double) (zeta :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
   Q(x) for the lognormal distribution with parameters
   zeta and sigma.")

;;; Examples and unit test
(save-test lognormal
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect
	    (sample rng :lognormal :zeta 1.0d0 :sigma 2.0d0)))
  (lognormal-pdf 1.2d0 1.0d0 2.0d0)
  (lognormal-P 1.2d0 1.0d0 2.0d0)
  (lognormal-Q 1.2d0 1.0d0 2.0d0)
  (lognormal-Pinv 0.3413288272347352d0 1.0d0 2.0d0)
  (lognormal-Qinv 0.6586711727652649d0 1.0d0 2.0d0))
