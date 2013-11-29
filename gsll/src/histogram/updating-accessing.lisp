;; Updating and accessing histogram elements.
;; Liam Healy, Mon Jan  1 2007 - 14:43
;; Time-stamp: <2010-06-30 21:02:26EDT updating-accessing.lisp>
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

(in-package :gsl)

;;; /usr/include/gsl/gsl_histogram.h
;;; /usr/include/gsl/gsl_histogram2d.h

(defmfun increment (histogram value &optional weight)
  ("gsl_histogram_increment" "gsl_histogram_accumulate")
  ((((mpointer histogram) :pointer) (value :double))
   (((mpointer histogram) :pointer) (value :double) (weight :double)))
  :documentation
  "Update the histogram by adding the weight
   (which defaults to 1.0) to the
   bin whose range contains the coordinate x. 

   If x lies in the valid range of the histogram then the function
   returns zero to indicate success.  If x is less than the lower
   limit of the histogram then the function issues a warning input-domain, and
   none of bins are modified.  Similarly, if the value of x is greater
   than or equal to the upper limit of the histogram then the function
   issues a warning input-domain, and none of the bins are modified.  The error
   handler is not called, however, since it is often necessary to compute
   histograms for a small range of a larger dataset, ignoring the values
   outside the range of interest.")

(defmfun grid:gref ((histogram histogram) &rest indices)
  "gsl_histogram_get"
  (((mpointer histogram) :pointer) ((first indices) sizet))
  :definition :method 
  :c-return :double
  :documentation			; FDL
  "Return the contents of the i-th bin of the histogram.
   If i lies outside the valid range of indices for the
   histogram then an error (input-domain) is signalled.")

(defmfun range (histogram i)
  "gsl_histogram_get_range"
  (((mpointer histogram) :pointer) (i sizet)
   (lower (:pointer :double)) (upper (:pointer :double)))
  :documentation			; FDL
  "Find the upper and lower range limits of the i-th
   bin of the histogram.  If the index i is valid then the
   corresponding range limits are stored in lower and upper.
   The lower limit is inclusive (i.e. events with this coordinate are
   included in the bin) and the upper limit is exclusive (i.e. events with
   the coordinate of the upper limit are excluded and fall in the
   neighboring higher bin, if it exists).
   If i lies outside the valid range of indices for
   the histogram, then the error input-domain is signalled.")

(defmfun max-range (histogram)
  "gsl_histogram_max"
  (((mpointer histogram) :pointer))
  :c-return :double
  :documentation			; FDL
  "The maximum upper range limit of the histogram.")

(defmfun min-range (histogram)
  "gsl_histogram_min"
  (((mpointer histogram) :pointer))
  :c-return :double
  :documentation			; FDL
  "The minimum lower range limit of the histogram.")

(defmfun bins (histogram)
  "gsl_histogram_bins"
  (((mpointer histogram) :pointer))
  :c-return sizet
  :documentation			; FDL
  "The number of bins in the histogram.")

(defmfun reset (histogram)
  "gsl_histogram_reset"
  (((mpointer histogram) :pointer))
  :c-return :void
  :documentation			; FDL
  "Reset all the bins in the histogram to zero.")

(defmfun histogram-find (histogram x-value &optional y-value)
  ("gsl_histogram_find" "gsl_histogram2d_find")
  ((((mpointer histogram) :pointer) (x-value :double) (bin (:pointer sizet)))
   (((mpointer histogram) :pointer) (x-value :double) (y-value :double)
   (xbin (:pointer sizet)) (ybin (:pointer sizet))))
  :documentation			; FDL
  "Finds the bin number which covers the coordinate value in
   the histogram.  The bin is located using a binary search. The
   search includes an optimization for histograms with uniform
   range, and will return the correct bin immediately in this
   case.  If the value is found in the range of the histogram
   then the function returns the index.  If value lies outside
   the valid range of the histogram then the error input-domain is
   signalled.")

#|
 (let ((histo (make-histogram 10)))		; should be a gsl-warning here, how to check?
     (set-ranges-uniform histo 0.0d0 10.0d0)
     (increment histo -2.0d0))
|#
;;; Examples and unit test

(save-test histogram
   ;; The first one gives a warning while compiling in SBCL,
   ;; should only give a warning while runnin.
 (let ((histo (make-histogram 10)))
     (set-ranges-uniform histo 0.0d0 10.0d0)
     (increment histo -2.0d0))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (grid:gref histo 1))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (grid:gref histo 2))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (grid:gref histo 6))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (grid:gref histo 16))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (values (min-range histo) (max-range histo)))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (bins histo))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (histogram-find histo 5.5d0)))
