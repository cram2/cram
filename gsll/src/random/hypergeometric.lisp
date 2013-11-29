;; Hypergeometric distribution
;; Liam Healy, Sat Nov 25 2006 - 16:00
;; Time-stamp: <2010-01-17 10:24:31EST hypergeometric.lisp>
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
    ((generator random-number-generator) (type (eql :hypergeometric))
     &key n1 n2 tt)
  "gsl_ran_hypergeometric"
  (((mpointer generator) :pointer) (n1 :uint) (n2 :uint)(tt :uint))
  :definition :method
  :c-return :uint
  :documentation			; FDL
  "A random integer from the hypergeometric
   distribution.  The probability distribution for hypergeometric
   random variates is
   p(k) =  C(n_1, k) C(n_2, t - k) / C(n_1 + n_2, t)
   where C(a,b) = a!/(b!(a-b)!) and 
   t <= n_1 + n_2.  The domain of k is 
   max(0,t-n_2), ..., min(t,n_1).
   If a population contains n_1 elements of ``type 1'' and
   n_2 elements of ``type 2'' then the hypergeometric
   distribution gives the probability of obtaining k elements of
   ``type 1'' in t samples from the population without
   replacement.")

(defmfun hypergeometric-pdf (k n1 n2 tt)
  "gsl_ran_hypergeometric_pdf" ((k :uint) (n1 :uint) (n2 :uint)(tt :uint))
  :c-return :double
  :documentation			; FDL
  "The probability p(k) of obtaining k
   from a hypergeometric distribution with parameters n1, n2,
   tt, using the formula given in #'sample :hypergeometric.")

(defmfun hypergeometric-P (k n1 n2 tt)
  "gsl_cdf_hypergeometric_P" ((k :uint) (n1 :uint) (n2 :uint)(tt :uint))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions P(k) for the
   hypergeometric distribution with parameters n1, n2 and tt.")

(defmfun hypergeometric-Q (k n1 n2 tt)
  "gsl_cdf_hypergeometric_Q"  ((k :uint) (n1 :uint) (n2 :uint)(tt :uint))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions Q(k) for the
   hypergeometric distribution with parameters n1, n2, and tt.")

;;; Examples and unit test
(save-test hypergeometric-randist
  (let ((rng (make-random-number-generator +mt19937+ 0)))
     (loop for i from 0 to 10
	   collect
	   (sample rng :hypergeometric :n1 3 :n2 6 :tt 3)))
  (hypergeometric-pdf 0 2 6 3)
  (hypergeometric-P 1 2 6 3)
  (hypergeometric-Q 1 2 6 3))
