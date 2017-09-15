;; The Gumbel type 2 random number distribution
;; Liam Healy, Sun Oct 29 2006
;; Time-stamp: <2010-01-17 10:23:29EST gumbel2.lisp>
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
    ((generator random-number-generator) (type (eql :gumbel2)) &key a b)
  "gsl_ran_gumbel2"
  (((mpointer generator) :pointer) (a :double) (b :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the Type-2 Gumbel
   distribution, p(x) dx = a b x^{-a-1} \exp(-b x^{-a}) dx
   for 0 < x < \infty.")

(defmfun gumbel2-pdf (x a b)
  "gsl_ran_gumbel2_pdf" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for a Type-2 Gumbel distribution with parameters a and b,
   using the formula given in #'sample :gumbel2.")

(defmfun gumbel2-P (x a b)
  "gsl_cdf_gumbel2_P" ((x :double) (a :double) (b :double))
  :c-return :double			; FDL
  :documentation "The cumulative distribution functions
  P(x) for the Type-2 Gumbel distribution with
  parameters a and b.")

(defmfun gumbel2-Q (x a b)
  "gsl_cdf_gumbel2_Q" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  Q(x) for the Type-2 Gumbel distribution with
  parameters a and b.")

(defmfun gumbel2-Pinv (P a b)
  "gsl_cdf_gumbel2_Pinv" ((P :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
  P(x) for the Type-2 Gumbel distribution with
  parameters a and b.")

(defmfun gumbel2-Qinv (Q a b)
  "gsl_cdf_gumbel2_Qinv" ((Q :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
  Q(x) for the Type-2 Gumbel distribution with
  parameters a and b.")

;;; Examples and unit test
(save-test gumbel2
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect
	    (sample rng :gumbel2 :a 1.0d0 :b 2.0d0)))
  (gumbel2-pdf 5.0d0 1.0d0 2.0d0)
  (gumbel2-P 10.0d0 1.0d0 2.0d0)
  (gumbel2-Q 10.0d0 1.0d0 2.0d0)
  (gumbel2-Pinv 0.8187307530779818d0 1.0d0 2.0d0)
  (gumbel2-Qinv 0.18126924692201815d0 1.0d0 2.0d0))
