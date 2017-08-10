;; Forward FFT.
;; Sumant Oemrawsingh, Sat Oct 31 2009 - 23:48
;; Time-stamp: <2012-01-13 12:01:31EST forward.lisp>
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
;; /usr/include/gsl/gsl_fft_halfcomplex.h
;; /usr/include/gsl/gsl_fft_halfcomplex_float.h

;;;;****************************************************************************
;;;; Forward transformation
;;;;****************************************************************************

;;; Forward transformations where the size is a power of 2
(defmfun forward-fourier-transform-radix2 ((vector vector) &key (stride 1))
  (double-float "gsl_fft_real_radix2_transform"
   single-float "gsl_fft_real_float_radix2_transform"
   complex-double-float "gsl_fft_complex_radix2_forward"
   complex-single-float "gsl_fft_complex_float_radix2_forward")
  (((grid:foreign-pointer vector) :pointer) (stride :sizet) ((floor (size vector) stride) :sizet))
  :definition :generic
  :element-types :float-complex
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index forward-fourier-transform
  :documentation
  "Forward FFT on a vector for which (floor length stride) is a power of 2.")

(defmfun forward-fourier-transform-nonradix2
    ((vector vector) &key (stride 1)
     (wavetable (make-fft-wavetable element-type (floor (size vector) stride)))
     (workspace (make-fft-workspace element-type (floor (size vector) stride))))
  (double-float "gsl_fft_real_transform"
		single-float "gsl_fft_real_float_transform"
		complex-double-float "gsl_fft_complex_forward"
		complex-single-float "gsl_fft_complex_float_forward")
  (((grid:foreign-pointer vector) :pointer) (stride :sizet) ((floor (size vector) stride) :sizet)
   ((mpointer wavetable) :pointer) ((mpointer workspace) :pointer))
  :definition :generic
  :element-types :float-complex
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index forward-fourier-transform
  :documentation
  "Forward FFT on a vector for which (floor length stride) is not a power of 2.")

;;;;****************************************************************************
;;;; Half-complex  
;;;;****************************************************************************

(defmfun forward-fourier-transform-halfcomplex-radix2
    ((vector vector) &key (stride 1))
  ("gsl_fft_halfcomplex" :type "_radix2_transform")
  (((grid:foreign-pointer vector) :pointer) (stride :sizet) ((floor (size vector) stride) :sizet))
  :definition :generic
  :element-types :float
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index forward-fourier-transform
  :documentation
  "Forward FFT on a vector for which (floor length stride) is a power of 2,
   in half complex form.")

(defmfun forward-fourier-transform-halfcomplex-nonradix2
    ((vector vector) &key (stride 1)
     (wavetable (make-fft-wavetable element-type (floor (size vector) stride) t))
     (workspace (make-fft-workspace element-type (floor (size vector) stride))))
  ("gsl_fft_halfcomplex" :type "_transform")
  (((grid:foreign-pointer vector) :pointer) (stride :sizet) ((floor (size vector) stride) :sizet)
   ((mpointer wavetable) :pointer) ((mpointer workspace) :pointer))
  :definition :generic
  :element-types :float
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index forward-fourier-transform
  :documentation
  "Forward FFT on a vector for which (floor length stride) is not a power of 2,
   in half complex form.")

;;;;****************************************************************************
;;;; Decimation-in-frequency forward FFT
;;;;****************************************************************************

(defmfun forward-fourier-transform-dif-radix2 ((vector vector) &key (stride 1))
  ("gsl_fft" :type "_radix2_dif_forward")
  (((grid:foreign-pointer vector) :pointer) (stride :sizet) ((floor (size vector) stride) :sizet))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index forward-fourier-transform
  :documentation
  "Forward decimation-in-frequency FFT on a vector for which (floor
  length stride) is a power of 2.")

;;;;****************************************************************************
;;;; Unified function forward-fourier-transform
;;;;****************************************************************************

;; Utility to determine if a vector can use a radix-2 transform.
(defun power-of-2-p (num)
  "The integer is a power of 2."
  (= 1 (logcount num)))

;;; This selects among the 14 forward FFT functions that GSL defines.
(export 'forward-fourier-transform)
(defun forward-fourier-transform
    (vector &rest args
     &key half-complex decimation-in-frequency (stride 1) non-radix-2 &allow-other-keys)
  "Perform a forward fast Fourier transform on the given vector. If
  the length of the vector is not a power of 2, and the user has a
  suitable wavetable and/or workspace, these can be supplied as
  keyword arguments.  If the (real) vector is in half-complex form,
  then the key argument :half-complex should be non-NIL. If the
  length of the vector is a power of 2, use of a non-radix-2 transform
  can be forced."
  (let ((pass-on-args (copy-list args)))
    (remf pass-on-args :half-complex)
    (remf pass-on-args :decimation-in-frequency)
    (remf pass-on-args :non-radix-2)
    (if (and (not non-radix-2) (power-of-2-p (floor (size vector) stride)))
	(if half-complex
	    (apply 'forward-fourier-transform-halfcomplex-radix2
		   vector pass-on-args)
	    (if decimation-in-frequency
		(apply 'forward-fourier-transform-dif-radix2 vector pass-on-args)
		(apply 'forward-fourier-transform-radix2 vector pass-on-args)))
	(if half-complex
	    (apply 'forward-fourier-transform-halfcomplex-nonradix2
		   vector pass-on-args)
	    (apply 'forward-fourier-transform-nonradix2 vector pass-on-args)))))
