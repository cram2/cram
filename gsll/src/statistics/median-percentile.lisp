;; Median and percentile
;; Liam Healy, Sun Dec 31 2006 - 13:19
;; Time-stamp: <2014-12-26 13:18:37EST median-percentile.lisp>
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

;;; To do: stride other than 1 when that information is availble from
;;; the vector.

(defmfun median ((sorted-data vector))
  ("gsl_stats" :type "_median_from_sorted_data")
  (((grid:foreign-pointer sorted-data) :pointer) (1 :int) ((dim0 sorted-data) :sizet))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (sorted-data)
  :documentation			; FDL
  "The median value of sorted-data.  The elements of the array
   must be in ascending numerical order.  There are no checks to see
   whether the data are sorted, so the function #'sort should
   always be used first.
   When the dataset has an odd number of elements the median is the value
   of element (n-1)/2.  When the dataset has an even number of
   elements the median is the mean of the two nearest middle values,
   elements (n-1)/2 and n/2.  Since the algorithm for
   computing the median involves interpolation this function always returns
   a floating-point number, even for integer data types.")

(defmfun quantile ((sorted-data vector) fraction)
  ("gsl_stats" :type "_quantile_from_sorted_data")
  (((grid:foreign-pointer sorted-data) :pointer) (1 :int) ((dim0 sorted-data) :sizet)
   (fraction :double))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (sorted-data)
  :documentation			; FDL
  "A quantile value of sorted-data.  The
   elements of the array must be in ascending numerical order.  The
   quantile is determined by a fraction between 0 and 1.  For
   example, to compute the value of the 75th percentile
   'fraction should have the value 0.75.
   There are no checks to see whether the data are sorted, so the function
   #'sort should always be used first.
   \hbox{quantile} = (1 - \delta) x_i + \delta x_{i+1}
   where i is floor((n - 1)f) and \delta is (n-1)f - i.
   Thus the minimum value of the array (data[0*stride]) is given by
   'fraction equal to zero, the maximum value (data[(n-1)*stride]) is
   given by 'fraction equal to one and the median value is given by 'fraction
   equal to 0.5.  Since the algorithm for computing quantiles involves
   interpolation this function always returns a floating-point number, even
   for integer data types.")

;;; Examples and unit test

(save-test median-percentile
  (let ((vec #m(-3.21d0 1.0d0 12.8d0)))
     (median vec))
  (let ((vec #m(-18.0d0 -12.0d0 -3.21d0 0.5d0 1.0d0 2.7d0 12.8d0)))
     (quantile vec 0.75d0)))
