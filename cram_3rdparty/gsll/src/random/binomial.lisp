;; Binomial distribution
;; Liam Healy, Sat Nov 25 2006 - 16:00
;; Time-stamp: <2009-12-27 10:00:03EST binomial.lisp>
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

(defmfun binomial (generator p n)
  "gsl_ran_binomial"
  (((mpointer generator) :pointer) (p :double) (n :uint))
  :c-return :uint
  :documentation			; FDL
  "A random integer from the binomial distribution,
  the number of successes in n independent trials with probability
  p.  The probability distribution for binomial variates is,
  p(k) = {n! \over k! (n-k)!} p^k (1-p)^{n-k}
  0 <= k <= n.")

(defmfun binomial-pdf (k p n)
  "gsl_ran_binomial_pdf" ((k :uint) (p :double) (n :uint))
  :c-return :double
  :documentation			; FDL
  "The probability p(k) of obtaining k
   from a binomial distribution with parameters p and n, using
   the formula given in #'binomial.")

(defmfun binomial-P (k p n)
  "gsl_cdf_binomial_P" ((k :uint) (p :double) (n :uint))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(k) for the Binomial distribution with parameters p and n.")

(defmfun binomial-Q (k p n)
  "gsl_cdf_binomial_Q" ((k :uint) (p :double) (n :uint))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
   Q(k) for the Binomial distribution
   with parameters p and n.")

;;; Examples and unit test
(save-test binomial
  (let ((rng (make-random-number-generator +mt19937+ 0)))
     (loop for i from 0 to 10
	   collect
	   (binomial rng 0.4d0 12)))
  (binomial-pdf 5 0.4d0 12)
  (binomial-P 5 0.4d0 12)
  (binomial-Q 5 0.4d0 12))


