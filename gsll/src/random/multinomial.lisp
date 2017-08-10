;; Multinomial distribution
;; Liam Healy, Sat Nov 25 2006 - 16:00
;; Time-stamp: <2014-12-26 13:24:44EST multinomial.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2011, 2012, 2014 Liam M. Healy
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
(named-readtables:in-readtable :antik)

;;; /usr/include/gsl/gsl_randist.h

(defmfun sample
    ((generator random-number-generator) (type (eql :multinomial))
     &key sum probabilities
     (n (grid:make-foreign-array
	 '(signed-byte 32) :dimensions (dim0 probabilities))))
  "gsl_ran_multinomial"
  (((mpointer generator) :pointer) 
   ((dim0 probabilities) :sizet)
   (sum :sizet)
   ((grid:foreign-pointer probabilities) :pointer)
   ;; technically, n should be a uint array, but integers work
   ((grid:foreign-pointer n) :pointer))
  :definition :method
  :inputs (p)
  :outputs (n)
  :return (n)
  :c-return :void
  :documentation			; FDL
  "Returns an array n of (dim0 probabilities) random variates from a 
   multinomial distribution.  The sum of the array n is specified
   by sum.  The distribution function is
   P(n_1, n_2, ..., n_K) = 
   (N!/(n_1! n_2! ... n_K!)) p_1^n_1 p_2^n_2 ... p_K^n_K
   where (n_1, n_2, ..., n_K) are nonnegative integers with 
   sum_{k=1}^K n_k = N, and (p_1, p_2, ..., p_K)
   is a probability distribution with \sum p_i = 1.  
   If the array p[K] is not normalized then its entries will be
   treated as weights and normalized appropriately.
   Random variates are generated using the conditional binomial method (see
   C.S. David, \"The computer generation of multinomial random
   variates,\" Comp. Stat. Data Anal. 16 (1993) 205--217 for details).")

(defmfun multinomial-pdf (p n)
  "gsl_ran_multinomial_pdf"
  (((dim0 p) :sizet) ((grid:foreign-pointer p) :pointer) ((grid:foreign-pointer n) :pointer))
  :inputs (p n)
  :c-return :double
  :documentation			; FDL
  "Compute the probability P(n_1, n_2, ..., n_K)
   of sampling n[K] from a multinomial distribution 
   with parameters p[K], using the formula given for #'sample :multinomial.")

(defmfun multinomial-log-pdf (p n)
  "gsl_ran_multinomial_lnpdf"
  (((dim0 p) :sizet) ((grid:foreign-pointer p) :pointer) ((grid:foreign-pointer n) :pointer))
  :inputs (p n)
  :c-return :double
  :documentation			; FDL
  "Compute the natural logarithm of the probability P(n_1, n_2, ..., n_K)
   of sampling n[K] from a multinomial distribution 
   with parameters p[K], using the formula given for #'sample :multinomial.")

;;; Examples and unit test
(save-test multinomial
 (let ((rng (make-random-number-generator +mt19937+ 0))
       (p (grid:make-foreign-array
	   'double-float :initial-contents '(0.1d0 0.2d0 0.3d0 0.4d0))))
   (grid:copy-to (sample rng :multinomial :sum 8 :probabilities p)))
 (let ((p (grid:make-foreign-array
	   'double-float :initial-contents '(0.1d0 0.2d0 0.3d0 0.4d0)))
       (n #31m(5 0 1 2)))
   (multinomial-pdf p N))
 (let ((p (grid:make-foreign-array
	   'double-float :initial-contents '(0.1d0 0.2d0 0.3d0 0.4d0)))
       (n #31m(5 0 1 2)))
   (multinomial-log-pdf p n)))
