;; Exponential distribution
;; Liam Healy, Sat Sep  2 2006 - 19:04
;; Time-stamp: <2010-05-24 20:46:09EDT exponential.lisp>
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
    ((generator random-number-generator) (type (eql :exponential))
     &key mu)
  "gsl_ran_exponential"
  (((mpointer generator) :pointer) (mu :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the exponential distribution
   with mean mu. The distribution is
   p(x) dx = {1 \over \mu} \exp(-x/\mu) dx
   x >= 0.")

(defmfun exponential-pdf (x mu)
  "gsl_ran_exponential_pdf" ((x :double) (mu :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
  for an exponential distribution with mean mu, using the formula
  given for #'sample :exponential.")

(defmfun exponential-P (x mu)
  "gsl_cdf_exponential_P" ((x :double) (mu :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution function
   P(x) for the exponential distribution with mean mu.")

(defmfun exponential-Q (x mu)
  "gsl_cdf_exponential_Q" ((x :double) (mu :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution function
   Q(x) for the exponential distribution with mean mu.")

(defmfun exponential-Pinv (P mu)
  "gsl_cdf_exponential_Pinv" ((P :double) (mu :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution function
   P(x) for the exponential distribution with mean mu.")

(defmfun exponential-Qinv (Q mu)
  "gsl_cdf_exponential_Qinv" ((Q :double) (mu :double))
  :c-return :double
  :documentation 			; FDL
  "The inverse cumulative distribution function
   Q(x) for the exponential distribution with mean mu.")

;;; Examples and unit test
(save-test exponential
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect
	    (sample rng :exponential :mu 10.0d0)))
  ;; From randist/test.c
  ;;(testpdf (lambda (r) (exponential-pdf r 2.0d0)) :exponential :mu 2.0d0)
  (exponential-pdf 0.0d0 10.0d0)
  (exponential-p 1.0d0 2.0d0)
  (exponential-q 1.0d0 2.0d0)
  (exponential-pinv 0.3934693402873666d0 2.0d0)
  (exponential-qinv 0.6065306597126334d0 2.0d0))

