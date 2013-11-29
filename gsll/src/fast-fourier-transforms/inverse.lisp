;; Inverse FFT
;; Sumant Oemrawsingh, Sat Oct 24 2009 - 12:55
;; Time-stamp: <2010-06-27 18:14:00EDT inverse.lisp>
;;
;; Copyright 2009 Sumant Oemrawsingh, Liam M. Healy
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

;; /usr/include/gsl/gsl_fft_complex.h
;; /usr/include/gsl/gsl_fft_complex_float.h
;; /usr/include/gsl/gsl_fft_halfcomplex.h
;; /usr/include/gsl/gsl_fft_halfcomplex_float.h

;;;;****************************************************************************
;;;; Inverse transformation
;;;;****************************************************************************

;;; Inverse transformations where the size is a power of 2
(defmfun inverse-fourier-transform-radix2
  ((vector vector) &key (stride 1))
  ("gsl_fft" :type "_radix2_inverse")
  (((foreign-pointer vector) :pointer) (stride sizet) ((floor (size vector) stride) sizet))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index inverse-fourier-transform
  :documentation
  "Inverse FFT on a vector for which (floor length stride) is a power of 2.")

(defmfun inverse-fourier-transform-nonradix2
    ((vector vector) &key (stride 1)
     (wavetable (make-fft-wavetable element-type (floor (size vector) stride)))
     (workspace (make-fft-workspace element-type (floor (size vector) stride))))
  ("gsl_fft" :type "_inverse")
  (((foreign-pointer vector) :pointer) (stride sizet) ((floor (size vector) stride) sizet)
   ((mpointer wavetable) :pointer) ((mpointer workspace) :pointer))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index inverse-fourier-transform
  :documentation
  "Inverse FFT on a complex vector for which (floor length stride) is
  not a power of 2.")

;;;;****************************************************************************
;;;; Half-complex  
;;;;****************************************************************************

(defmfun inverse-fourier-transform-halfcomplex-radix2
    ((vector vector) &key (stride 1))
  ("gsl_fft_halfcomplex" :type "_radix2_inverse")
  (((foreign-pointer vector) :pointer) (stride sizet) ((floor (size vector) stride) sizet))
  :definition :generic
  :element-types :float
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index inverse-fourier-transform
  :documentation
  "Inverse FFT on a vector for which (floor length stride) is a power of 2,
   in half complex form.")

(defmfun inverse-fourier-transform-halfcomplex-nonradix2
    ((vector vector) &key (stride 1)
     (wavetable (make-fft-wavetable element-type (floor (size vector) stride) t))
     (workspace (make-fft-workspace element-type (floor (size vector) stride))))
  ("gsl_fft_halfcomplex" :type "_inverse")
  (((foreign-pointer vector) :pointer) (stride sizet) ((floor (size vector) stride) sizet)
   ((mpointer wavetable) :pointer) ((mpointer workspace) :pointer))
  :definition :generic
  :element-types :float
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index inverse-fourier-transform
  :documentation
  "Inverse FFT on a vector for which (floor length stride) is not a power of 2,
   in half complex form.")

;;;;****************************************************************************
;;;; Decimation-in-frequency inverse FFT
;;;;****************************************************************************

(defmfun inverse-fourier-transform-dif-radix2
  ((vector vector) &key (stride 1))
  ("gsl_fft" :type "_radix2_dif_inverse")
  (((foreign-pointer vector) :pointer) (stride sizet) ((floor (size vector) stride) sizet))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index inverse-fourier-transform
  :documentation
  "Inverse decimation-in-frequency FFT on a vector for which (floor
  length stride) is a power of 2.")

;;;;****************************************************************************
;;;; Unified function inverse-fourier-transform
;;;;****************************************************************************

;;; This selects among the 10 inverse FFT functions that GSL defines.
(export 'inverse-fourier-transform)
(defun inverse-fourier-transform
    (vector &rest args
     &key decimation-in-frequency (stride 1) &allow-other-keys)
  "Perform a inverse fast Fourier transform on the given vector. If
  the length of the vector is not a power of 2, and the user has a
  suitable wavetable and/or workspace, these can be supplied as
  keyword arguments.  If the vector is real, it is assumed to be
  in half-complex form."
  (let ((pass-on-args (copy-list args)))
    (remf pass-on-args :half-complex)
    (remf pass-on-args :decimation-in-frequency)
    (if (power-of-2-p (floor (size vector) stride))
	(if (subtypep (element-type vector) 'real)
	    (apply 'inverse-fourier-transform-halfcomplex-radix2
		   vector pass-on-args)
	    (if decimation-in-frequency
		(apply 'inverse-fourier-transform-dif-radix2 vector pass-on-args)
		(apply 'inverse-fourier-transform-radix2 vector pass-on-args)))
	(if (subtypep (element-type vector) 'real)
	    (apply 'inverse-fourier-transform-halfcomplex-nonradix2
		   vector pass-on-args)
	    (apply 'inverse-fourier-transform-nonradix2 vector pass-on-args)))))
