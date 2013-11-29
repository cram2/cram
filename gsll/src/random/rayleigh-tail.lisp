;; Rayleigh tail distribution
;; Liam Healy, Sat Sep 30 2006
;; Time-stamp: <2010-01-17 10:31:22EST rayleigh-tail.lisp>
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
    ((generator random-number-generator) (type (eql :rayleigh-tail))
     &key a sigma)
  "gsl_ran_rayleigh_tail"
  (((mpointer generator) :pointer) (a :double) (sigma :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the tail of the Rayleigh
  distribution with scale parameter sigma and a lower limit of
  a.  The distribution is
  p(x) dx = {x \over \sigma^2} \exp ((a^2 - x^2) /(2 \sigma^2)) dx
  for x > a.")

(defmfun rayleigh-tail-pdf (x a sigma)
  "gsl_ran_rayleigh_tail_pdf" ((x :double) (a :double) (sigma :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for a Rayleigh tail distribution with scale parameter sigma and
   lower limit a, using the formula given in #'sample :rayleigh-tail.")

;;; Examples and unit test
(save-test rayleigh-tail
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect (sample rng :rayleigh-tail :a 1.0d0 :sigma 10.0d0)))
  (rayleigh-tail-pdf 0.25d0 -2.0d0 2.0d0))
