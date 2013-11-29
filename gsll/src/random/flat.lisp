;; Flat distribution
;; Liam Healy, Oct  7 2006
;; Time-stamp: <2010-01-17 10:13:12EST flat.lisp>
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

(defmfun sample
    ((generator random-number-generator) (type (eql :flat)) &key a b)
  "gsl_ran_flat"
  (((mpointer generator) :pointer) (a :double) (b :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the flat (uniform)
   distribution from a to b.  The distribution is
   p(x) dx = {1 \over (b-a)} dx
   if a <= x < b, and 0 otherwise.")

(defmfun flat-pdf (x a b)
  "gsl_ran_flat_pdf" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for a uniform distribution from a to b, using the formula
   given for #'sample :flat.")

(defmfun flat-P (x a b)
  "gsl_cdf_flat_P" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
   P(x) for a uniform distribution from a to b.")

(defmfun flat-Q (x a b)
  "gsl_cdf_flat_Q" ((x :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
   Q(x) for a uniform distribution from a to b.")

(defmfun flat-Pinv (P a b)
  "gsl_cdf_flat_Pinv" ((P :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
   P(x) for a uniform distribution from a to b.")

(defmfun flat-Qinv (Q a b)
  "gsl_cdf_flat_Qinv" ((Q :double) (a :double) (b :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
   Q(x) for a uniform distribution from a to b.")

;;; Examples and unit test
(save-test flat
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect
	    (sample rng :flat :a 1.0d0 :b 2.0d0)))
  (flat-pdf 1.2d0 1.0d0 2.0d0)
  (flat-P 1.2d0 1.0d0 2.0d0)
  (flat-Q 1.2d0 1.0d0 2.0d0)
  (flat-Pinv 0.19999999999999996d0 1.0d0 2.0d0)
  (flat-Qinv 0.8d0 1.0d0 2.0d0))
