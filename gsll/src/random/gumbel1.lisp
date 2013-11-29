;; The Gumbel type 1 random number distribution
;; Liam Healy, Sun Oct 29 2006
;; Time-stamp: <2010-01-17 10:22:56EST gumbel1.lisp>
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
    ((generator random-number-generator) (type (eql :gumbel1)) &key a b)
  "gsl_ran_gumbel1"
  (((mpointer generator) :pointer) (a :double) (b :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the Type-1 Gumbel
   distribution,
   p(x) dx = a b \exp(-(b \exp(-ax) + ax)) dx
   for -\infty < x < \infty.")

(defmfun gumbel1-pdf (x a b)
  "gsl_ran_gumbel1_pdf" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
  for a Type-1 Gumbel distribution with parameters a and b,
  using the formula given for #'sample :gumbel1.")

(defmfun gumbel1-P (x a b)
  "gsl_cdf_gumbel1_P" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(x) for the Type-1 Gumbel distribution with
  parameters a and b.")

(defmfun gumbel1-Q (x a b)
  "gsl_cdf_gumbel1_Q" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  Q(x) for the Type-1 Gumbel distribution with
  parameters a and b.")

(defmfun gumbel1-Pinv (P a b)
  "gsl_cdf_gumbel1_Pinv" ((P :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
  P(x) for the Type-1 Gumbel distribution with
  parameters a and b.")

(defmfun gumbel1-Qinv (Q a b)
  "gsl_cdf_gumbel1_Qinv" ((Q :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
  Q(x) for the Type-1 Gumbel distribution with
  parameters a and b.")

;;; Examples and unit test
(save-test gumbel1
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect (sample rng :gumbel1 :a 1.0d0 :b 2.0d0)))
  (gumbel1-pdf 0.1d0 1.0d0 2.0d0)
  (gumbel1-P 0.1d0 1.0d0 2.0d0)
  (gumbel1-Q 0.1d0 1.0d0 2.0d0)
  (gumbel1-Pinv 0.1637073598773166d0 1.0d0 2.0d0)
  (gumbel1-Qinv 0.8362926401226833d0 1.0d0 2.0d0))


