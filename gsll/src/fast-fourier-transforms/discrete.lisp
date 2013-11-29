;; Discrete Fourier Transforms
;; Liam Healy 2009-11-07 14:24:07EST
;; Time-stamp: <2010-06-29 22:15:25EDT discrete.lisp>
;;
;; Copyright 2009 Liam M. Healy
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

;;; These functions are not documented in GSL and appear to be present
;;; only to provide a check on the FFT routines.

(in-package :gsl)

;; /usr/include/gsl/gsl_dft_complex.h
;; /usr/include/gsl/gsl_dft_complex_float.h

(defmfun forward-discrete-fourier-transform
    ((vector vector)
     &key (stride 1)
     (result (grid:make-foreign-array element-type :dimensions (dimensions vector))))
  ("gsl_dft" :type "_forward")
  (((foreign-pointer vector) :pointer) (stride sizet) ((floor (size vector) stride) sizet)
   ((foreign-pointer result) :pointer))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (result)
  :return (result)
  :documentation
  "Forward discrete Fourier transform provided to check the FFT
  routines.")

(defmfun backward-discrete-fourier-transform
    ((vector vector)
     &key (stride 1)
     (result (grid:make-foreign-array element-type :dimensions (dimensions vector))))
  ("gsl_dft" :type "_backward")
  (((foreign-pointer vector) :pointer) (stride sizet) ((floor (size vector) stride) sizet)
   ((foreign-pointer result) :pointer))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (result)
  :return (result)
  :documentation
  "Backward discrete Fourier transform provided to check the FFT
  routines.")

(defmfun inverse-discrete-fourier-transform
    ((vector vector)
     &key (stride 1)
     (result (grid:make-foreign-array element-type :dimensions (dimensions vector))))
  ("gsl_dft" :type "_inverse")
  (((foreign-pointer vector) :pointer) (stride sizet) ((floor (size vector) stride) sizet)
   ((foreign-pointer result) :pointer))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (result)
  :return (result)
  :documentation
  "Inverse discrete Fourier transform provided to check the FFT
  routines.")

(defmfun discrete-fourier-transform
    ((vector vector)
     &key (stride 1)
     (result (grid:make-foreign-array element-type :dimensions (dimensions vector))))
  ("gsl_dft" :type "_transform")
  (((foreign-pointer vector) :pointer) (stride sizet) ((floor (size vector) stride) sizet)
   ((foreign-pointer result) :pointer))
  :definition :generic
  :element-types :complex
  :inputs (vector)
  :outputs (result)
  :return (result)
  :documentation
  "Discrete Fourier transform in selectable direction provided to
  check the FFT routines.")
