;; Logistic distribution
;; Liam Healy, Sat Oct  7 2006 - 16:13
;; Time-stamp: <2010-01-17 10:27:24EST logistic.lisp>
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
    ((generator random-number-generator) (type (eql :logistic)) &key a)
  "gsl_ran_logistic"
  (((mpointer generator) :pointer) (a :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the logistic distribution.  The distribution function is
   p(x) dx = { \exp(-x/a) \over a (1 + \exp(-x/a))^2 } dx
   for -\infty < x < +\infty.")

(defmfun logistic-pdf (x a)
  "gsl_ran_logistic_pdf" ((x :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for a logistic distribution with scale parameter a, using the
   formula given in #'sample :logistic.")

(defmfun logistic-P (x a)
  "gsl_cdf_logistic_P" ((x :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(x) for the logistic distribution with scale parameter a.")

(defmfun logistic-Q (x a)
  "gsl_cdf_logistic_Q" ((x :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  Q(x) for the logistic distribution with scale parameter a.")

(defmfun logistic-Pinv (P a)
  "gsl_cdf_logistic_Pinv" ((P :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
  P(x) for the logistic distribution with scale parameter a.")

(defmfun logistic-Qinv (Q a)
  "gsl_cdf_logistic_Qinv" ((Q :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
   Q(x) for the logistic distribution with scale parameter a.")

;;; Examples and unit test
(save-test logistic
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect
	    (sample rng :logistic :a 10.0d0)))
  (logistic-pdf 0.5d0 1.0d0)
  (logistic-P 0.5d0 1.0d0)
  (logistic-Q 0.5d0 1.0d0)
  (logistic-Pinv 0.6224593312018546d0 1.0d0)
  (logistic-Qinv 0.37754066879814546d0 1.0d0))
