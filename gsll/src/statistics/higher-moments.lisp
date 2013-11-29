;; Skewness and kurtosis.
;; Liam Healy, Sun Dec 31 2006 - 14:20
;; Time-stamp: <2010-06-27 18:14:02EDT higher-moments.lisp>
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
;;; To do: stride other than 1 when that information is availble from
;;; the vector.

(defmfun skewness ((data vector) &optional mean standard-deviation)
  (("gsl_stats" :type "_skew")
   ("gsl_stats" :type "_skew_m_sd"))
  ((((foreign-pointer data) :pointer) (1 :int) ((dim0 data) sizet))
   (((foreign-pointer data) :pointer) (1 :int) ((dim0 data) sizet)
    (mean :double) (standard-deviation :double)))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (data)
  :documentation			; FDL
  "The skewness of data, defined as skew = (1/N) \sum ((x_i -
  \Hat\mu)/\Hat\sigma)^3 where x_i are the elements of the dataset
  data.  The skewness measures the asymmetry of the tails of a
  distribution.  If mean and standard deviation are supplied, compute
  skewness of the dataset data using the given values skew = (1/N)
  \sum ((x_i - mean)/sd)^3.  This is useful if you have
  already computed the mean and standard deviation of data and want to
  avoid recomputing them.")

(defmfun kurtosis ((data vector) &optional mean standard-deviation)
  (("gsl_stats" :type "_kurtosis")
   ("gsl_stats" :type "_kurtosis_m_sd"))
  ((((foreign-pointer data) :pointer) (1 :int) ((dim0 data) sizet))
   (((foreign-pointer data) :pointer) (1 :int) ((dim0 data) sizet)
    (mean :double) (standard-deviation :double)))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (data)
  :documentation			; FDL
  "The kurtosis of data defined as
   kurtosis = ((1/N) \sum ((x_i - \Hat\mu)/\Hat\sigma)^4)  - 3
   The kurtosis measures how sharply peaked a distribution is,
   relative to its width.  The kurtosis is normalized to zero
   for a gaussian distribution.")

(defmfun weighted-skewness
    ((data vector) (weights vector) &optional mean standard-deviation)
  (("gsl_stats" :type "_wskew")
   ("gsl_stats" :type "_wskew_m_sd"))
  ((((foreign-pointer weights) :pointer) (1 :int)
    ((foreign-pointer data) :pointer) (1 :int)
    ((dim0 data) sizet))
   (((foreign-pointer weights) :pointer) (1 :int)
    ((foreign-pointer data) :pointer) (1 :int)
    ((dim0 data) sizet)
    (mean :double) (standard-deviation :double)))
  :definition :generic
  :element-types :float
  :c-return :double
  :inputs (data weights)
  :documentation			; FDL
  "The weighted skewness of the dataset.
   skew = (\sum w_i ((x_i - xbar)/\sigma)^3) / (\sum w_i).")

(defmfun weighted-kurtosis
    ((data vector) (weights vector) &optional mean standard-deviation)
  (("gsl_stats" :type "_wkurtosis")
   ("gsl_stats" :type "_wkurtosis_m_sd"))
  ((((foreign-pointer weights) :pointer) (1 :int)
    ((foreign-pointer data) :pointer) (1 :int)
    ((dim0 data) sizet))
   (((foreign-pointer weights) :pointer) (1 :int)
    ((foreign-pointer data) :pointer) (1 :int)
    ((dim0 data) sizet)
    (mean :double) (standard-deviation :double)))
  :definition :generic
  :element-types :float
  :c-return :double
  :inputs (data weights)
  :documentation			; FDL
  "The weighted kurtosis of the dataset.
   kurtosis = ((\sum w_i ((x_i - xbar)/sigma)^4) / (\sum w_i)) - 3.")

;;; Examples and unit test

(save-test higher-moments
  (let ((vec #m(-3.21d0 1.0d0 12.8d0)))
      (let* ((mean (mean vec))
	     (sd (standard-deviation vec mean)))
	(list
	 (skewness vec)
	 (skewness vec mean sd)
	 (kurtosis vec)
	 (kurtosis vec mean sd)))))
