;; Exponential distribution
;; Liam Healy, Sun Sep 17 2006
;; Time-stamp: <2010-05-24 19:58:32EDT laplace.lisp>
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
    ((generator random-number-generator) (type (eql :laplace)) &key a)
  "gsl_ran_laplace"
  (((mpointer generator) :pointer) (a :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the Laplace distribution with width a.
   The distribution is
   p(x) dx = {1 \over 2 a}  \exp(-|x/a|) dx
   for -\infty < x < \infty.")

(defmfun laplace-pdf (x a)
  "gsl_ran_laplace_pdf" ((x :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for a Laplace distribution with width a, using the formula
   given for #'sample :laplace.")

(defmfun laplace-P (x a)
  "gsl_cdf_laplace_P" ((x :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution function
   P(x) for the laplace distribution with width a.")

(defmfun laplace-Q (x a)
  "gsl_cdf_laplace_Q" ((x :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution function
   Q(x) for the laplace distribution with width a.")

(defmfun laplace-Pinv (P a)
  "gsl_cdf_laplace_Pinv" ((P :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution function
   P(x) for the laplace distribution with width a.")

(defmfun laplace-Qinv (Q a)
  "gsl_cdf_laplace_Qinv" ((Q :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution function
   Q(x) for the laplace distribution with width a.")

;;; Examples and unit test
(save-test laplace
 (testpdf (lambda (r) (laplace-pdf r 2.75d0)) :laplace :a 2.75d0)
 (laplace-p 1.0d0 2.0d0)
 (laplace-q 1.0d0 2.0d0)
 (laplace-pinv 0.6967346701436833d0 2.0d0)
 (laplace-qinv 0.3032653298563167d0 2.0d0))
