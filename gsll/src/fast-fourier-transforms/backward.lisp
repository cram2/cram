;; Backward FFT
;; Sumant Oemrawsingh, Sat Oct 24 2009 - 12:55
;; Time-stamp: <2012-01-13 12:01:32EST backward.lisp>
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

;; /usr/include/gsl/gsl_fft_complex.h
;; /usr/include/gsl/gsl_fft_complex_float.h
;; /usr/include/gsl/gsl_fft_halfcomplex.h
;; /usr/include/gsl/gsl_fft_halfcomplex_float.h

;;;;****************************************************************************
;;;; Backward transformation
;;;;****************************************************************************

;;; Backward transformations where the size is a power of 2
(defmfun backward-fourier-transform-radix2
  ((vector vector) &key (stride 1))
  ("gsl_fft" :type "_radix2_backward")
  (((grid:foreign-pointer vector) :pointer) (stride :sizet) ((floor (size vector) stride) :sizet))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index backward-fourier-transform
  :documentation
  "Backward FFT on a vector for which (floor length stride) is a power of 2.")

(defmfun backward-fourier-transform-nonradix2
    ((vector vector) &key (stride 1)
     (wavetable (make-fft-wavetable element-type (floor (size vector) stride)))
     (workspace (make-fft-workspace element-type (floor (size vector) stride))))
  ("gsl_fft" :type "_backward")
  (((grid:foreign-pointer vector) :pointer) (stride :sizet) ((floor (size vector) stride) :sizet)
   ((mpointer wavetable) :pointer) ((mpointer workspace) :pointer))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index backward-fourier-transform
  :documentation
  "Backward FFT on a complex vector for which (floor length stride) is
  not a power of 2.")

;;;;****************************************************************************
;;;; Half-complex  
;;;;****************************************************************************

(defmfun backward-fourier-transform-halfcomplex-radix2
    ((vector vector) &key (stride 1))
  ("gsl_fft_halfcomplex" :type "_radix2_backward")
  (((grid:foreign-pointer vector) :pointer) (stride :sizet) ((floor (size vector) stride) :sizet))
  :definition :generic
  :element-types :float
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index backward-fourier-transform
  :documentation
  "Backward FFT on a vector for which (floor length stride) is a power of 2,
   in half complex form.")

(defmfun backward-fourier-transform-halfcomplex-nonradix2
    ((vector vector) &key (stride 1)
     (wavetable (make-fft-wavetable element-type (floor (size vector) stride) t))
     (workspace (make-fft-workspace element-type (floor (size vector) stride))))
  ("gsl_fft_halfcomplex" :type "_backward")
  (((grid:foreign-pointer vector) :pointer) (stride :sizet) ((floor (size vector) stride) :sizet)
   ((mpointer wavetable) :pointer) ((mpointer workspace) :pointer))
  :definition :generic
  :element-types :float
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index backward-fourier-transform
  :documentation
  "Backward FFT on a vector for which (floor length stride) is not a power of 2,
   in half complex form.")

;;;;****************************************************************************
;;;; Decimation-in-frequency backward FFT
;;;;****************************************************************************

(defmfun backward-fourier-transform-dif-radix2
  ((vector vector) &key (stride 1))
  ("gsl_fft" :type "_radix2_dif_backward")
  (((grid:foreign-pointer vector) :pointer) (stride :sizet) ((floor (size vector) stride) :sizet))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (vector)
  :return (vector)
  :export nil
  :index backward-fourier-transform
  :documentation
  "Backward decimation-in-frequency FFT on a vector for which (floor
  length stride) is a power of 2.")

;;;;****************************************************************************
;;;; Unified function backward-fourier-transform
;;;;****************************************************************************

;;; This selects among the 10 backward FFT functions that GSL defines.
(export 'backward-fourier-transform)
(defun backward-fourier-transform
    (vector &rest args
     &key decimation-in-frequency (stride 1) non-radix-2 &allow-other-keys)
  "Perform a backward fast Fourier transform on the given vector. If
  the length of the vector is not a power of 2, and the user has a
  suitable wavetable and/or workspace, these can be supplied as
  keyword arguments. If the vector is real, it is assumed to be in
  half-complex form. If the length of the vector is a power of 2, use
  of a non-radix-2 transform can be forced."
  (let ((pass-on-args (copy-list args)))
    (remf pass-on-args :half-complex)
    (remf pass-on-args :decimation-in-frequency)
    (remf pass-on-args :non-radix-2)
    (if (and (not non-radix-2) (power-of-2-p (floor (size vector) stride)))
	(if (subtypep (grid:element-type vector) 'real)
	    (apply 'backward-fourier-transform-halfcomplex-radix2
		   vector pass-on-args)
	    (if decimation-in-frequency
		(apply 'backward-fourier-transform-dif-radix2 vector pass-on-args)
		(apply 'backward-fourier-transform-radix2 vector pass-on-args)))
	(if (subtypep (grid:element-type vector) 'real)
	    (apply 'backward-fourier-transform-halfcomplex-nonradix2
		   vector pass-on-args)
	    (apply 'backward-fourier-transform-nonradix2 vector pass-on-args)))))
