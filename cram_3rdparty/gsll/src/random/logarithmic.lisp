;; Logarithmic distribution
;; Liam Healy, Sat Nov 25 2006 - 16:00
;; Time-stamp: <2010-01-17 10:26:45EST logarithmic.lisp>
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
    ((generator random-number-generator) (type (eql :logarithmic))
     &key probability)
  "gsl_ran_logarithmic"
  (((mpointer generator) :pointer) (probability :double))
  :definition :method
  :c-return :uint
  :documentation			; FDL
  "A random integer from the logarithmic distribution.
   The probability distribution for logarithmic random variates
   is p(k) = {-1 \over \log(1-p)} {\left( p^k \over k \right)}
   for k >= 1.")

(defmfun logarithmic-pdf (k p)
  "gsl_ran_logarithmic_pdf" ((k :uint) (p :double))
  :c-return :double
  :documentation
  "The probability p(k) of obtaining k
   from a logarithmic distribution with probability parameter p,
   using the formula given in #'sample :logarithmic.")

;;; Examples and unit test
(save-test logarithmic
  (let ((rng (make-random-number-generator +mt19937+ 0)))
     (loop for i from 0 to 10
	   collect
	   (sample rng :logarithmic :probability 0.9d0)))
  (logarithmic-pdf 2 0.4d0))
