;; Shuffling and sampling
;; Liam Healy, Sat Dec  2 2006 - 18:40
;; Time-stamp: <2014-12-26 13:24:44EST shuffling-sampling.lisp>
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

;;; These are currently defined only for vectors.

(defmfun sample
    ((generator random-number-generator) (type (eql :shuffle))
     &key base)
  "gsl_ran_shuffle"
  (((mpointer generator) :pointer)
   ((grid:foreign-pointer base) :pointer) ((dim0 base) :sizet) ((grid:element-size base) :sizet))
  :definition :method
  :inputs (base)
  :outputs (base)
  :c-return :void
  :documentation			; FDL
  "Randomly shuffle the order of n objects, each of
   size size, stored in the array base[0...n-1].  The
   output of the random number generator r is used to produce the
   permutation.  The algorithm generates all possible n!
   permutations with equal probability, assuming a perfect source of random
   numbers.")

(defmfun sample
    ((generator random-number-generator) (type (eql :choose-random))
     &key src (dest (dim0 src))
     &aux
     (destarr
      (if (integerp dest)
	  (grid:make-foreign-array (grid:element-type src) :dimensions dest)
	  dest)))
  "gsl_ran_choose"
  (((mpointer generator) :pointer)
   ((grid:foreign-pointer destarr) :pointer) ((dim0 destarr) :sizet)
   ((grid:foreign-pointer src) :pointer) ((dim0 src) :sizet) ((grid:element-size src) :sizet))
  :definition :method
  :inputs (src)
  :outputs (destarr)
  :documentation			; FDL
  "Fill the array destarr[k] with k objects taken randomly from the n
   elements of the array src[0...n-1].  The output of the random
   number generator r is used to make the selection.  The algorithm
   ensures all possible samples are equally likely, assuming a perfect
   source of randomness.

   The objects are sampled without replacement, thus each object can
   only appear once in destarr[k].  It is required that k be less
   than or equal to n.  The objects in destarr will be in the
   same relative order as those in src.  You will need to call
   with :shuffle if you want to randomize the order.")

(defmfun sample
    ((generator random-number-generator) (type (eql :random-sample))
     &key src (dest (dim0 src))
     &aux
     (destarr
      (if (integerp dest)
	  (grid:make-foreign-array (grid:element-type src) :dimensions dest)
	  dest)))
  "gsl_ran_sample"
  (((mpointer generator) :pointer)
   ((grid:foreign-pointer destarr) :pointer) ((dim0 destarr) :sizet)
   ((grid:foreign-pointer src) :pointer) ((dim0 src) :sizet) ((grid:element-size src) :sizet))
  :definition :method
  :inputs (src)
  :outputs (destarr)
  :c-return :void
  :documentation
  "Like :choose-random, but samples k items from the original array of
   n items src with replacement, so the same object can appear more
   than once in the output sequence dest.  There is no requirement
   that k be less than n in this case.")

;;; Examples and unit test
(save-test shuffling-sampling
 (let ((rng (make-random-number-generator +mt19937+ 0))
       (v1 #31m(1 2 3 4 5 6 7 8)))
   (grid:copy-to (sample rng :shuffle :base v1)))
 (let ((rng (make-random-number-generator +mt19937+ 0))
       (v1 #31m(1 2 3 4 5 6 7 8)))
   (grid:copy-to (sample rng :choose-random :src v1 :dest 4)))
 (let ((rng (make-random-number-generator +mt19937+ 0))
       (v1 #31m(1 2 3 4 5 6 7 8)))
   (grid:copy-to (sample rng :random-sample :src v1 :dest 10))))
