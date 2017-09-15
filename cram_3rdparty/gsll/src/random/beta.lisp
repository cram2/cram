;; Beta distribution
;; Liam Healy, Sat Sep 30 2006
;; Time-stamp: <2010-05-24 20:11:32EDT beta.lisp>
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
    ((generator random-number-generator) (type (eql :beta)) &key a b)
  "gsl_ran_beta"
  (((mpointer generator) :pointer) (a :double) (b :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the beta distribution.  The distribution function is
   p(x) dx = {\Gamma(a+b) \over \Gamma(a) \Gamma(b)} x^{a-1} (1-x)^{b-1} dx
   0 <= x <= 1.")

(defmfun beta-pdf (x a b)
  "gsl_ran_beta_pdf" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for a beta distribution with parameters a and b, using the
   formula given in #'sample :beta.")

(defmfun beta-P (x a b)
  "gsl_cdf_beta_P" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(x) for the beta distribution with parameters a and b.")

(defmfun beta-Q (x a b)
  "gsl_cdf_beta_Q" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  Q(x) for the beta distribution with parameters a and b.")

(defmfun beta-Pinv (P a b)
  "gsl_cdf_beta_Pinv" ((P :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
  P(x) for the beta distribution with parameters a and b.")

(defmfun beta-Qinv (Q a b)
  "gsl_cdf_beta_Qinv" ((Q :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
   Q(x) for the beta distribution with parameters a and b.")

;;; Examples and unit test
(save-test
 beta
 ;;(testpdf (lambda (r) (beta-pdf r 2.0d0 3.0d0)) :beta :a 2.0d0 :b 3.0d0)
 (let ((rng (make-random-number-generator +mt19937+ 0)))
   (loop for i from 0 to 10
      collect
      (sample rng :beta :a 1.0d0 :b 2.0d0)))
 (beta-pdf 0.1d0 1.0d0 2.0d0)
 (beta-P 0.1d0 1.0d0 2.0d0)
 (beta-Q 0.1d0 1.0d0 2.0d0)
 (beta-Pinv 0.19d0 1.0d0 2.0d0)
 (beta-Qinv 0.81d0 1.0d0 2.0d0))



