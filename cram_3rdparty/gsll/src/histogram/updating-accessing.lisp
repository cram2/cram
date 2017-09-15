;; Updating and accessing histogram elements.
;; Liam Healy, Mon Jan  1 2007 - 14:43
;; Time-stamp: <2016-01-20 16:51:15EST updating-accessing.lisp>
;;
;; Copyright 2007, 2008, 2009, 2011, 2012, 2014, 2015, 2016 Liam M. Healy
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

;;; Missing: 2D functions/methods; the 1D functions will need to be made into methods if not already.
;;; http://www.gnu.org/s/gsl/manual/html_node/Updating-and-accessing-2D-histogram-elements.html

(export 'increment)
(defgeneric increment (histogram value &optional weight)
  (:documentation
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
   outside the range of interest."))

(defmfun increment ((histogram histogram) value &optional weight)
  ("gsl_histogram_increment" "gsl_histogram_accumulate")
  ((((mpointer histogram) :pointer) (value :double))
   (((mpointer histogram) :pointer) (value :double) (weight :double)))
  :definition :method)

(defmfun increment ((histogram histogram2d) values &optional weight)
  ("gsl_histogram2d_increment" "gsl_histogram2d_accumulate")
  ((((mpointer histogram) :pointer) ((first values) :double) ((second values) :double))
   (((mpointer histogram) :pointer) ((first values) :double) ((second values) :double) (weight :double)))
  :definition :method)

(defmfun grid:aref ((histogram histogram) &rest indices)
  "gsl_histogram_get"
  (((mpointer histogram) :pointer) ((first indices) :sizet))
  :definition :method 
  :c-return :double
  :index grid:aref
  :documentation			; FDL
  "Return the contents of the i-th bin of the histogram.
   If i lies outside the valid range of index for the
   histogram then an error (input-domain) is signalled.")

(defmfun grid:aref ((histogram histogram2d) &rest indices)
  "gsl_histogram2d_get"
  (((mpointer histogram) :pointer) ((first indices) :sizet) ((second indices) :sizet))
  :definition :method 
  :c-return :double
  :index grid:aref
  :documentation
  "Return the contents of the i-th, j-th bin of the 2D histogram. If either index lies outside the valid range of index for the histogram then an error (input-domain) is signalled.")

(defgeneric range (histogram i)
  (:documentation
   "Find the upper and lower range limits of the i-th
   bin of the histogram.  If the index i is valid then the
   corresponding range limits are stored in lower and upper.
   The lower limit is inclusive (i.e. events with this coordinate are
   included in the bin) and the upper limit is exclusive (i.e. events with
   the coordinate of the upper limit are excluded and fall in the
   neighboring higher bin, if it exists).
   If i lies outside the valid range of indices for
   the histogram, then the error input-domain is signalled.")
  (:method ((histogram histogram2d) i)
    (list
     (funcall
      (defmfun nil (histogram i)
	"gsl_histogram2d_get_xrange"
	(((mpointer histogram) :pointer) (i :sizet)))
      histogram i)
     (funcall
      (defmfun nil (histogram i)
	"gsl_histogram2d_get_yrange"
	(((mpointer histogram) :pointer) (i :sizet)))
      histogram i))))

(map-name 'range "gsl_histogram2d_get_xrange")
(map-name 'range "gsl_histogram2d_get_yrange")
(export 'range)

(defmfun range ((histogram histogram) i)
  "gsl_histogram_get_range"
  (((mpointer histogram) :pointer) (i :sizet)
   (lower (:pointer :double)) (upper (:pointer :double)))
  :definition :method)

(defgeneric max-range (histogram)
  (:documentation "The maximum upper range limit(s) of the histogram.")
  (:method ((histogram histogram2d))
    (list
     (funcall
      (defmfun nil (histogram)
	"gsl_histogram2d_xmax"
	(((mpointer histogram) :pointer))
	:c-return :double)
      histogram)
     (funcall
      (defmfun nil (histogram)
	"gsl_histogram2d_ymax"
	(((mpointer histogram) :pointer))
	:c-return :double)
      histogram))))

(map-name 'max-range "gsl_histogram2d_xmax")
(map-name 'max-range "gsl_histogram2d_ymax")
(export 'max-range)

(defmfun max-range ((histogram histogram))
  "gsl_histogram_max"
  (((mpointer histogram) :pointer))
  :c-return :double
  :definition :method)

(defgeneric min-range (histogram)
  (:documentation "The minimum lower range limit(s) of the histogram.")
  (:method ((histogram histogram2d))
    (list
     (funcall
      (defmfun nil (histogram)
	"gsl_histogram2d_xmin"
	(((mpointer histogram) :pointer))
	:c-return :double)
      histogram)
     (funcall
      (defmfun nil (histogram)
	"gsl_histogram2d_ymin"
	(((mpointer histogram) :pointer))
	:c-return :double)
      histogram))))

(map-name 'min-range "gsl_histogram2d_xmin")
(map-name 'min-range "gsl_histogram2d_ymin")
(export 'min-range)

(defmfun min-range ((histogram histogram))
  "gsl_histogram_min"
  (((mpointer histogram) :pointer))
  :c-return :double
  :definition :method)

(defmfun grid:dimensions ((histogram histogram))
  "gsl_histogram_bins"
  (((mpointer histogram) :pointer))
  :definition :method
  :c-return (dim :sizet)
  :return ((list dim))
  :documentation			; FDL
  "The number of bins in the histogram.")

(defmethod grid:dimensions ((histogram histogram2d))
  (list
   (funcall
    (defmfun nil (histogram)
      "gsl_histogram2d_nx"
      (((mpointer histogram) :pointer))
      :c-return :sizet)
    histogram)
   (funcall
    (defmfun nil (histogram)
      "gsl_histogram2d_ny"
      (((mpointer histogram) :pointer))
      :c-return :sizet)
    histogram)))

(map-name 'grid:dimensions "gsl_histogram2d_nx")
(map-name 'grid:dimensions "gsl_histogram2d_ny")

(defmfun set-zero ((histogram histogram))
  "gsl_histogram_reset"
  (((mpointer histogram) :pointer))
  :definition :method
  :c-return :void
  :documentation			; FDL
  "Reset all the bins in the histogram to zero.")

(defmfun set-zero ((histogram histogram2d))
  "gsl_histogram2d_reset"
  (((mpointer histogram) :pointer))
  :definition :method
  :c-return :void
  :documentation			; FDL
  "Reset all the bins in the histogram to zero.")

(defmfun histogram-find (histogram x-value &optional y-value)
  ("gsl_histogram_find" "gsl_histogram2d_find")
  ((((mpointer histogram) :pointer) (x-value :double) (bin (:pointer :sizet)))
   (((mpointer histogram) :pointer) (x-value :double) (y-value :double)
   (xbin (:pointer :sizet)) (ybin (:pointer :sizet))))
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
   (grid:aref histo 1))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (grid:aref histo 2))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (grid:aref histo 6))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (grid:aref histo 16))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (values (min-range histo) (max-range histo)))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (grid:dimensions histo))
 (let ((histo (make-histogram 10)))
   (set-ranges-uniform histo 0.0d0 10.0d0)
   (increment histo 2.7d0)
   (increment histo 6.9d0 2.0d0)
   (histogram-find histo 5.5d0)))
