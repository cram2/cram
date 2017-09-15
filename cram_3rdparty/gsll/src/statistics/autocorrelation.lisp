;; Autocorrelation
;; Liam Healy, Sun Dec 31 2006 - 13:19
;; Time-stamp: <2014-12-26 13:18:38EST autocorrelation.lisp>
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

(defmfun autocorrelation ((data vector) &optional mean)
  (("gsl_stats" :type "_lag1_autocorrelation")
   ("gsl_stats" :type "_lag1_autocorrelation_m"))
  ((((grid:foreign-pointer data) :pointer) (1 :int) ((dim0 data) :sizet))
   (((grid:foreign-pointer data) :pointer) (1 :int) ((dim0 data) :sizet)
    (mean :double)))
  :definition :generic
  :element-types :no-complex
  :c-return :double
  :inputs (data)
  :documentation			; FDL
  "The lag-1 autocorrelation of the dataset data.
  a_1 = {\sum_{i = 1}^{n} (x_{i} - \Hat\mu) (x_{i-1} - \Hat\mu)
  \over
  \sum_{i = 1}^{n} (x_{i} - \Hat\mu) (x_{i} - \Hat\mu)}.")

;;; Examples and unit test

(save-test autocorrelation
  (let ((vec #m(-3.21d0 1.0d0 12.8d0)))
      (let ((mean (mean vec)))
	(list
	 (autocorrelation vec)
	 (autocorrelation vec mean)))))
