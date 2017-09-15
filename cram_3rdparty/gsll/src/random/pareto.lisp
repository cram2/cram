;; Pareto distribution
;; Liam Healy, Sat Oct  8 2006 - 21:23
;; Time-stamp: <2010-01-17 10:30:11EST pareto.lisp>
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
    ((generator random-number-generator) (type (eql :pareto)) &key a b)
  "gsl_ran_pareto"
  (((mpointer generator) :pointer) (a :double) (b :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the Pareto distribution of order a.
   The distribution function is
   p(x) dx = (a/b) / (x/b)^{a+1} dx
   x >= b.")

(defmfun pareto-pdf (x a b)
  "gsl_ran_pareto_pdf" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for a Pareto distribution with exponent a and scale b, using
   the formula given in #'sample :pareto.")

(defmfun pareto-P (x a b)
  "gsl_cdf_pareto_P" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(x) for the Pareto distribution with exponent a and scale b.")

(defmfun pareto-Q (x a b)
  "gsl_cdf_pareto_Q" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  Q(x) for the Pareto distribution with exponent a and scale b.")

(defmfun pareto-Pinv (P a b)
  "gsl_cdf_pareto_Pinv" ((P :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
  P(x) for the Pareto distribution with exponent a and scale b.")

(defmfun pareto-Qinv (Q a b)
  "gsl_cdf_pareto_Qinv" ((Q :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
   Q(x) for the Pareto distribution with exponent a and scale b.")

;;; Examples and unit test
(save-test pareto
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect
	    (sample rng :pareto :a 1.0d0 :b 2.0d0)))
  (pareto-pdf 1.5d0 1.3d0 1.0d0)
  (pareto-P 3.5d0 1.3d0 2.0d0)
  (pareto-Q 3.5d0 1.3d0 2.0d0)
  (pareto-Pinv 0.5168849835182453d0 1.3d0 2.0d0)
  (pareto-Qinv 0.4831150164817547d0 1.3d0 2.0d0))
