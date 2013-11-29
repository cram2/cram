;; Poisson distribution
;; Liam Healy, Sat Nov 25 2006 - 16:00
;; Time-stamp: <2010-01-17 10:30:48EST poisson.lisp>
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
    ((generator random-number-generator) (type (eql :poisson)) &key mu)
  "gsl_ran_poisson"
  (((mpointer generator) :pointer) (mu :double))
  :definition :method
  :c-return :uint
  :documentation			; FDL
  "A random integer from the Poisson distribution with mean mu.
   The probability distribution for Poisson variates is
   p(k) = {\mu^k \over k!} \exp(-\mu)
   k >= 0.")

(defmfun poisson-pdf (k mu)
  "gsl_ran_poisson_pdf" ((k :uint) (mu :double))
  :c-return :double
  :documentation			; FDL
  "The probability p(k) of obtaining k
   from a Poisson distribution with mean mu using the formula
   given in #'sample :poisson.")

(defmfun poisson-P (k mu)
  "gsl_cdf_poisson_P" ((k :uint) (mu :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(k) for the Poisson distribution with parameter mu.")

(defmfun poisson-Q (k mu)
  "gsl_cdf_poisson_Q" ((k :uint) (mu :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  Q(k) for the Poisson distribution with parameter mu.")

;;; Examples and unit test
(save-test poisson
  (let ((rng (make-random-number-generator +mt19937+ 0)))
     (loop for i from 0 to 10
	   collect
	   (sample rng :poisson :mu 10.0d0)))
  (poisson-pdf 8 10.0d0)
  (poisson-P 8 10.0d0)
  (poisson-Q 8 10.0d0))
