;; Landau distribution
;; Liam Healy, Sat Sep 30 2006
;; Time-stamp: <2010-01-17 10:24:59EST landau.lisp>
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
    ((generator random-number-generator) (type (eql :landau)) &key)
  "gsl_ran_landau"
  (((mpointer generator) :pointer))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the Landau distribution.  The
   probability distribution for Landau random variates is defined
   analytically by the complex integral,
   {1 \over {2 \pi i}} \int_{c-i\infty}^{c+i\infty} ds\, \exp(s \log(s) + x s) 
   For numerical purposes it is more convenient to use the following
   equivalent form of the integral,
   p(x) = (1/\pi) \int_0^\infty dt \exp(-t \log(t) - x t) \sin(\pi t).")

(defmfun landau-pdf (x)
  "gsl_ran_landau_pdf" ((x :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for the Landau distribution using an approximation to the formula given
   in #'sample :landau.")

;;; Examples and unit test
(save-test landau
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect (sample rng :landau)))
  (landau-pdf 0.25d0))
