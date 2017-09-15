;; Gamma distribution
;; Liam Healy, Sat Sep 30 2006
;; Time-stamp: <2010-01-17 10:14:35EST gamma.lisp>
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
    ((generator random-number-generator) (type (eql :gamma)) &key a b)
  "gsl_ran_gamma"
  (((mpointer generator) :pointer) (a :double) (b :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the gamma distribution.
   The distribution function is
   p(x) dx = {1 \over \Gamma(a) b^a} x^{a-1} e^{-x/b} dx
   for x > 0. The gamma distribution with an integer parameter a
   is known as the Erlang distribution.  The variates are computed using
   the algorithms from Knuth (vol 2).")

(defmfun sample
    ((generator random-number-generator) (type (eql :gamma-mt)) &key a b)
  "gsl_ran_gamma_mt"
  (((mpointer generator) :pointer) (a :double) (b :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A gamma variate using the Marsaglia-Tsang fast gamma method.")

(defmfun gamma-pdf (x a b)
  "gsl_ran_gamma_pdf" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for a gamma distribution with parameters a and b, using the
   formula given in #'sample :gamma.")

(defmfun gamma-P (x a b)
  "gsl_cdf_gamma_P" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(x) for the Gamma distribution with parameters a and b.")

(defmfun gamma-Q (x a b)
  "gsl_cdf_gamma_Q" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  Q(x) for the Gamma distribution with parameters a and b.")

(defmfun gamma-Pinv (P a b)
  "gsl_cdf_gamma_Pinv" ((P :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
  P(x) for the Gamma distribution with parameters a and b.")

(defmfun gamma-Qinv (Q a b)
  "gsl_cdf_gamma_Qinv" ((Q :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
   Q(x) for the Gamma distribution with parameters a and b.")

;;; Examples and unit test
(save-test gamma-randist
 (let ((rng (make-random-number-generator +mt19937+ 0)))
   (loop for i from 0 to 10
	 collect
	 (sample rng :gamma :a 1.0d0 :b 2.0d0)))
 (let ((rng (make-random-number-generator +mt19937+ 0)))
   (loop for i from 0 to 10
	 collect
	 (sample rng :gamma-mt :a 1.0d0 :b 2.0d0)))
 (gamma-pdf 0.1d0 1.0d0 2.0d0)
 (gamma-P 0.1d0 1.0d0 2.0d0)
 (gamma-Q 0.1d0 1.0d0 2.0d0)
 (gamma-Pinv 0.048770575499286005d0 1.0d0 2.0d0)
 (gamma-Qinv 0.951229424500714d0 1.0d0 2.0d0))
