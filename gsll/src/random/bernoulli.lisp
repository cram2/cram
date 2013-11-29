;; Bernoulli distribution
;; Liam Healy, Sat Nov 25 2006 - 16:59
;; Time-stamp: <2010-01-17 10:06:06EST bernoulli.lisp>
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

(defmfun sample
    ((generator random-number-generator) (type (eql :bernoulli))
     &key probability)
  "gsl_ran_bernoulli"
  (((mpointer generator) :pointer) (probability :double))
  :definition :method
  :c-return :uint
  :documentation			; FDL
  "Returns either 0 or 1, the result of a Bernoulli trial
   with probability p.  The probability distribution for
   a Bernoulli trial is
   p(0) = 1 - p
   p(1) = p.")

(defmfun bernoulli-pdf (k p)
  "gsl_ran_bernoulli_pdf" ((k :uint) (p :double))
  :c-return :double
  :documentation			; FDL
  "The probability p(k) of obtaining
  k from a Bernoulli distribution with probability parameter
  p, using the formula given in #'sample :bernoulli.")

;;; Examples and unit test
(save-test bernoulli
  (let ((rng (make-random-number-generator +mt19937+ 0)))
     (loop for i from 0 to 10
	   collect
	   (sample rng :bernoulli :probability 0.5d0)))
  (bernoulli-pdf 0 0.5d0))
