;; Gaussian tail distribution
;; Liam Healy, Mon Aug 21 2006 - 21:52
;; Time-stamp: <2010-01-17 10:17:01EST gaussian-tail.lisp>
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
    ((generator random-number-generator) (type (eql :gaussian-tail))
     &key a sigma)
  "gsl_ran_gaussian_tail"
  (((mpointer generator) :pointer) (a :double) (sigma :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "Random variates from the upper tail of a Gaussian
   distribution with standard deviation sigma.  The values returned
   are larger than the lower limit a, which must be positive.  The
   method is based on Marsaglia's famous rectangle-wedge-tail algorithm (Ann. 
   Math. Stat. 32, 894--899 (1961)), with this aspect explained in Knuth, v2,
   3rd ed, p139,586 (exercise 11).
   The probability distribution for Gaussian tail random variates is,
   p(x) dx = {1 \over N(a;\sigma) \sqrt{2 \pi \sigma^2}}
                  \exp (- x^2 / 2\sigma^2) dx
   for x > a where N(a;\sigma) is the normalization constant,
   N(a;\sigma) = (1/2) erfc(a / sqrt(2 sigma^2)).")

(defmfun gaussian-tail-pdf (x a sigma)
  "gsl_ran_gaussian_tail_pdf" ((x :double) (a :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
  for a Gaussian tail distribution with standard deviation sigma and
  lower limit a, using the formula given for gaussian-tail.")

(defmfun sample
    ((generator random-number-generator) (type (eql :ugaussian-tail))
     &key a)
  "gsl_ran_ugaussian_tail"
  (((mpointer generator) :pointer) (a :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "Equivalent to gaussian-tail with sigma=1.")

(defmfun ugaussian-tail-pdf (x a)
  "gsl_ran_ugaussian_tail_pdf" ((x :double) (a :double))
  :c-return :double
  :documentation			; FDL
  "Equivalent to gaussian-tail-pdf with sigma=1.")

;;; Examples and unit test
(save-test
 gaussian-tail
 (let ((rng (make-random-number-generator +mt19937+ 0)))
   (loop for i from 0 to 10
	 collect
	 (sample rng :gaussian-tail :a 50.0d0 :sigma 10.0d0)))
 (gaussian-tail-pdf 52.0d0 50.0d0 10.0d0)
 (let ((rng (make-random-number-generator +mt19937+ 0)))
     (loop for i from 0 to 10
	   collect
	   (sample rng :ugaussian-tail :a 5.0d0)))
 (ugaussian-tail-pdf 5.2d0 5.0d0))
