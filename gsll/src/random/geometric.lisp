;; Geometric distribution
;; Liam Healy, Sat Nov 25 2006 - 16:00
;; Time-stamp: <2010-01-17 10:22:14EST geometric.lisp>
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
    ((generator random-number-generator) (type (eql :geometric))
     &key probability)
  "gsl_ran_geometric"
  (((mpointer generator) :pointer) (probability :double))
  :definition :method
  :c-return :uint
  :documentation			; FDL
  "A random integer from the geometric distribution,
   the number of independent trials with probability p until the
   first success.  The probability distribution for geometric variates
   is p(k) =  p (1-p)^{k-1} for k >= 1.
   Note that the distribution begins with k=1 with this
   definition.  There is another convention in which the exponent k-1
   is replaced by k.")

(defmfun geometric-pdf (k p)
  "gsl_ran_geometric_pdf" ((k :uint) (p :double))
  :c-return :double
  :documentation			; FDL
  "The probability p(k) of obtaining k
   from a geometric distribution with probability parameter p, using
   the formula given in #'sample :geometric.")

(defmfun geometric-P (k p)
  "gsl_cdf_geometric_P" ((k :uint) (p :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(k) for the geometric distribution with parameter p.")

(defmfun geometric-Q (k p)
  "gsl_cdf_geometric_Q" ((k :uint) (p :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  Q(k) for the geometric distribution with parameters p.")

;;; Examples and unit test
(save-test geometric
  (let ((rng (make-random-number-generator +mt19937+ 0)))
     (loop for i from 0 to 10
	   collect
	   (sample rng :geometric :probability 0.4d0)))
  (geometric-pdf 2 0.4d0)
  (geometric-P 2 0.4d0)
  (geometric-Q 2 0.4d0))
