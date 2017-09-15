;; Cauchy distribution
;; Liam Healy, Sat Sep 30 2006
;; Time-stamp: <2010-01-17 10:05:32EST cauchy.lisp>
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
    ((generator random-number-generator) (type (eql :cauchy)) &key a)
  "gsl_ran_cauchy"
  (((mpointer generator) :pointer) (a :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the Cauchy distribution with
   scale parameter a.  The probability distribution for Cauchy
   random variates is,
   p(x) dx = {1 \over a\pi (1 + (x/a)^2) } dx
   for x in the range -\infty to +\infty.  The Cauchy
   distribution is also known as the Lorentz distribution.")

(defmfun cauchy-pdf (x a)
  "gsl_ran_cauchy_pdf" ((x :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for a Cauchy distribution with scale parameter a, using the formula
   given for #'sample :cauchy.")

(defmfun cauchy-P (x a)
  "gsl_cdf_cauchy_P" ((x :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(x) for the Cauchy distribution with scale parameter a.")

(defmfun cauchy-Q (x a)
  "gsl_cdf_cauchy_Q" ((x :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  Q(x) for the Cauchy distribution with scale parameter a.")

(defmfun cauchy-Pinv (P a)
  "gsl_cdf_cauchy_Pinv" ((P :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
  P(x) for the Cauchy distribution with scale parameter a.")

(defmfun cauchy-Qinv (Q a)
  "gsl_cdf_cauchy_Qinv" ((Q :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
   Q(x) for the Cauchy distribution with scale parameter a.")

;;; Examples and unit test
(save-test cauchy
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect
	    (sample rng :cauchy :a 10.0d0)))
  (cauchy-pdf 0.0d0 10.0d0)
  (cauchy-P 1.0d0 2.0d0)
  (cauchy-Q 1.0d0 2.0d0)
  (cauchy-Pinv 0.6475836176504333d0 2.0d0)
  (cauchy-Qinv 0.35241638234956674d0 2.0d0))

