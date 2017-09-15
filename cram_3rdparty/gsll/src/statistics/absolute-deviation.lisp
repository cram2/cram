;; Absolute deviation
;; Liam Healy, Sun Dec 31 2006 - 13:19
;; Time-stamp: <2014-12-26 13:18:37EST absolute-deviation.lisp>
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

(defmfun absolute-deviation ((data vector) &optional mean)
  (("gsl_stats" :type "_absdev")
   ("gsl_stats" :type "_absdev_m"))
  ((((grid:foreign-pointer data) :pointer) (1 :int) ((dim0 data) :sizet))
   (((grid:foreign-pointer data) :pointer) (1 :int) ((dim0 data) :sizet)
    (mean :double)))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (data)
  :documentation			; FDL
  "The absolute deviation from the mean of data.  The absolute
  deviation from the mean is defined as absdev = (1/N) \sum |x_i -
  \Hat\mu| where x_i are the elements of the dataset data.  The
  absolute deviation from the mean provides a more robust measure of
  the width of a distribution than the variance.  If 'mean is not
  supplied, this function computes the mean of data via a call to
  #'mean.  With mean supplied, this function is useful if you have
  already computed the mean of data (and want to avoid recomputing
  it), or wish to calculate the absolute deviation relative to another
  value (such as zero, or the median).")

(defmfun weighted-absolute-deviation ((data vector) (weights vector) &optional mean)
  (("gsl_stats" :type "_wabsdev")
   ("gsl_stats" :type "_wabsdev_m"))
  ((((grid:foreign-pointer weights) :pointer) (1 :int)
    ((grid:foreign-pointer data) :pointer) (1 :int)
    ((dim0 data) :sizet))
   (((grid:foreign-pointer weights) :pointer) (1 :int)
    ((grid:foreign-pointer data) :pointer) (1 :int)
    ((dim0 data) :sizet) (mean :double)))
  :definition :generic
  :element-types :float
  :c-return :double
  :inputs (data weights)
  :documentation			; FDL
  "The weighted absolute deviation from the weighted
   mean, defined as
   absdev = (\sum w_i |x_i - \Hat\mu|) / (\sum w_i).")

;;; Examples and unit test

(save-test absolute-deviation
  (let ((vec #m(-3.21d0 1.0d0 12.8d0))
	 (weights #m(3.0d0 1.0d0 2.0d0)))
      (let ((mean (mean vec)))
	(list
	 (absolute-deviation vec)
	 (weighted-absolute-deviation vec weights)
	 (absolute-deviation vec mean)))))
