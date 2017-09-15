;; Unpack functions for FFT vectors.
;; Sumant Oemrawsingh, Sun Oct 25 2009 - 16:35
;; Time-stamp: <2012-01-13 12:01:29EST unpack.lisp>
;;
;; Copyright 2009, 2011 Sumant Oemrawsingh, Liam M. Healy
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

;; /usr/include/gsl/gsl_fft_real.h
;; /usr/include/gsl/gsl_fft_real_float.h
;; /usr/include/gsl/gsl_fft_complex.h
;; /usr/include/gsl/gsl_fft_complex_float.h

(defmfun fft-real-unpack
    ((vector vector)
     &key (stride 1)
     (output
	 (eltcase single-float
		  (grid:make-foreign-array '(complex single-float) :dimensions (size vector))
		  t
		  (grid:make-foreign-array '(complex double-float) :dimensions (size vector)))))
  ("gsl_fft_real" :type "_unpack")
  (((grid:foreign-pointer vector) :pointer)
   ((grid:foreign-pointer output) :pointer)
   (stride :sizet) ((floor (size vector) stride) :sizet))
  :definition :generic
  :element-types :float
  :inputs (vector output)
  :outputs (output)
  :return (output)
  :export nil
  :index unpack
  :documentation
  "This function converts a single real array into an equivalent complex
  array (with imaginary part set to zero), suitable for fft-complex
  routines.")

(defmfun fft-half-complex-radix2-unpack
    ((vector vector)
     &key (stride 1)
     (output
	 (eltcase single-float
		  (grid:make-foreign-array '(complex single-float) :dimensions (size vector))
		  t
		  (grid:make-foreign-array '(complex double-float) :dimensions (size vector)))))
  ("gsl_fft_halfcomplex" :type "_radix2_unpack")
  (((grid:foreign-pointer vector) :pointer)
   ((grid:foreign-pointer output) :pointer)
   (stride :sizet) ((floor (size vector) stride) :sizet))
  :definition :generic
  :element-types :float
  :inputs (vector output)
  :outputs (output)
  :return (output)
  :export nil
  :index unpack
  :documentation
  "Convert an array of half-complex coefficients as returned by
  real-fft-radix2-transform, into an ordinary complex array.")

(defmfun fft-half-complex-unpack
    ((vector vector)
     &key (stride 1)
     (output
	 (eltcase single-float
		  (grid:make-foreign-array '(complex single-float) :dimensions (size vector))
		  t
		  (grid:make-foreign-array '(complex double-float) :dimensions (size vector)))))
  ("gsl_fft_halfcomplex" :type "_unpack")
  (((grid:foreign-pointer vector) :pointer)
   ((grid:foreign-pointer output) :pointer)
   (stride :sizet) ((floor (size vector) stride) :sizet))
  :definition :generic
  :element-types :float
  :inputs (vector output)
  :outputs (output)
  :return (output)
  :export nil
  :index unpack
  :documentation
  "This function converts an array of half-complex coefficients as
  returned by fft-real-transform, into an ordinary complex array. It
  fills in the complex array using the symmetry z_k = z_{n-k}^* to
  reconstruct the redundant elements.")

(export 'unpack)
(defun unpack
    (vector &rest args &key (stride 1) (unpack-type 'real) &allow-other-keys)
  (let ((pass-on-args (copy-list args)))
    (remf pass-on-args :unpack-type)
    (if (eq unpack-type 'complex)
	(if (power-of-2-p (floor (size vector) stride))
	    (apply 'fft-half-complex-radix2-unpack vector pass-on-args)
	    (apply 'fft-half-complex-unpack vector pass-on-args))
	(apply 'fft-real-unpack vector pass-on-args))))
