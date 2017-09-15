;; Histogram operations
;; Liam Healy, Mon Jan  1 2007 - 16:47
;; Time-stamp: <2009-12-27 09:47:40EST operations.lisp>
;;
;; Copyright 2007, 2008, 2009 Liam M. Healy
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
;; $Id$

(in-package :gsl)

(export 'equal-bins-p)
(defgeneric equal-bins-p (histogram1 histogram2)
  (:documentation ;; FDL
   "Are all of the individual bin ranges of the two histograms are identical?"))

(defmfun equal-bins-p ((histogram1 histogram) (histogram2 histogram))
  "gsl_histogram_equal_bins_p"
  (((mpointer histogram1) :pointer) ((mpointer histogram2) :pointer))
  :definition :method
  :c-return :true-false)

(defmfun equal-bins-p ((histogram1 histogram2d) (histogram2 histogram2d))
  "gsl_histogram2d_equal_bins_p"
  (((mpointer histogram1) :pointer) ((mpointer histogram2) :pointer))
  :definition :method
  :c-return :true-false)

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun elt+ ((histogram1 histogram) (histogram2 histogram))
  "gsl_histogram_add"
  (((mpointer histogram1) :pointer) ((mpointer histogram2) :pointer))
  :definition :method
  :documentation			; FDL
  "Add the contents of the bins in histogram2 to the
   corresponding bins of histogram1 i.e. h'_1(i) =
   h_1(i) + h_2(i).  The two histograms must have identical bin
   ranges.")

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun elt+ ((histogram1 histogram2d) (histogram2 histogram2d))
  "gsl_histogram2d_add"
  (((mpointer histogram1) :pointer) ((mpointer histogram2) :pointer))
  :definition :method
  :documentation			; FDL
  "Add the contents of the bins in histogram2 to the
   corresponding bins of histogram1 i.e. h'_1(i) =
   h_1(i) + h_2(i).  The two histograms must have identical bin
   ranges.")

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun elt- ((histogram1 histogram) (histogram2 histogram))
  "gsl_histogram_sub"
  (((mpointer histogram1) :pointer) ((mpointer histogram2) :pointer))
  :definition :method
  :documentation			; FDL
  "Subtract the contents of the bins in histogram2 from the
   corresponding bins of histogram1 i.e. h'_1(i) = h_1(i) -
   h_2(i).  The two histograms must have identical bin ranges.")

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun elt- ((histogram1 histogram2d) (histogram2 histogram2d))
  "gsl_histogram2d_sub"
  (((mpointer histogram1) :pointer) ((mpointer histogram2) :pointer))
  :definition :method
  :documentation			; FDL
  "Subtract the contents of the bins in histogram2 from the
   corresponding bins of histogram1 i.e. h'_1(i) = h_1(i) -
   h_2(i).  The two histograms must have identical bin ranges.")

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun elt* ((histogram1 histogram) (histogram2 histogram))
  "gsl_histogram_mul"
  (((mpointer histogram1) :pointer) ((mpointer histogram2) :pointer))
  :definition :method
  :documentation			; FDL
  "Multiply the contents of the bins of histogram1 by the
   contents of the corresponding bins in histogram2
   i.e. h'_1(i) = h_1(i) * h_2(i).  The two histograms
   must have identical bin ranges.")

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun elt* ((histogram1 histogram2d) (histogram2 histogram2d))
  "gsl_histogram2d_mul"
  (((mpointer histogram1) :pointer) ((mpointer histogram2) :pointer))
  :definition :method
  :documentation			; FDL
  "Multiply the contents of the bins of histogram1 by the
   contents of the corresponding bins in histogram2
   i.e. h'_1(i) = h_1(i) * h_2(i).  The two histograms
   must have identical bin ranges.")

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun elt/ ((histogram1 histogram) (histogram2 histogram))
  "gsl_histogram_div"
  (((mpointer histogram1) :pointer) ((mpointer histogram2) :pointer))
  :definition :method
  :documentation			; FDL
  "Divide the contents of the bins of histogram1 by the
   contents of the corresponding bins in histogram2
   i.e. h'_1(i) = h_1(i) / h_2(i).  The two histograms
   must have identical bin ranges.")

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun elt/ ((histogram1 histogram2d) (histogram2 histogram2d))
  "gsl_histogram2d_div"
  (((mpointer histogram1) :pointer) ((mpointer histogram2) :pointer))
  :definition :method
  :documentation			; FDL
  "Divide the contents of the bins of histogram1 by the
   contents of the corresponding bins in histogram2
   i.e. h'_1(i) = h_1(i) / h_2(i).  The two histograms
   must have identical bin ranges.")

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun elt* ((histogram histogram) scale)
  "gsl_histogram_scale"
  (((mpointer histogram) :pointer) (scale :double))
  :definition :method
  :documentation			; FDL
  "Multiply the contents of the bins of histogram by the
   constant scale, i.e. h'_1(i) = h_1(i) * scale.")

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun elt* ((histogram histogram2d) scale)
  "gsl_histogram2d_scale"
  (((mpointer histogram) :pointer) (scale :double))
  :definition :method
  :documentation			; FDL
  "Multiply the contents of the bins of histogram by the
   constant scale, i.e. h'_1(i) = h_1(i) * scale.")

(export 'shift)
(defgeneric shift (histogram offset)
  (:documentation ;; FDL
   "Shift the contents of the bins of histogram h by the
   constant offset, i.e. h'_1(i) = h_1(i) + offset."))

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun shift ((histogram histogram) offset)
  "gsl_histogram_shift"
  (((mpointer histogram) :pointer) (offset :double))
  :definition :method)

;;; GSL documentation does not state what the return value for the
;;; C function means; assumed to be error code.
(defmfun shift ((histogram histogram2d) offset)
  "gsl_histogram2d_shift"
  (((mpointer histogram) :pointer) (offset :double))
  :definition :method)
