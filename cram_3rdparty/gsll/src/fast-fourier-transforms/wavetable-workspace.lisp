;; Wavetable and workspace object definitions.
;; Sumant Oemrawsingh, Sun Oct 25 2009 - 16:35
;; Time-stamp: <2012-01-13 12:01:32EST wavetable-workspace.lisp>
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

;;; Define the wavetables and workspaces needed for non-power-of-2
;;; dimension vectors.  These mobjects and their makers are not
;;; exported; instead, the user should call
;;; make-fft-wavetable and make-fft-workspace.

;;;;****************************************************************************
;;;; Real
;;;;****************************************************************************

;; /usr/include/gsl/gsl_fft_real.h
;; /usr/include/gsl/gsl_fft_real_float.h

(defmobject fft-real-wavetable-double-float
    "gsl_fft_real_wavetable" ((n :sizet))
  "structure that holds the factorization and trigonometric lookup tables for
  the mixed radix real fft algorithm"
  :export nil
  :documentation
  "These functions prepare trigonometric lookup tables for an FFT of size n
  real elements. The functions return a pointer to the newly allocated struct
  if no errors were detected, and a null pointer in the case of error. The
  length n is factorized into a product of subtransforms, and the factors and
  their trigonometric coefficients are stored in the wavetable. The
  trigonometric coefficients are computed using direct calls to sin and cos,
  for accuracy. Recursion relations could be used to compute the lookup table
  faster, but if an application performs many FFTs of the same length then
  computing the wavetable is a one-off overhead which does not affect the
  final throughput.

  The wavetable structure can be used repeatedly for any transform of the same
  length. The table is not modified by calls to any of the other FFT
  functions. The appropriate type of wavetable must be used for forward real
  or inverse half-complex transforms.")

(defmobject fft-real-wavetable-single-float
    "gsl_fft_real_wavetable_float" ((n :sizet))
  "structure that holds the factorization and trigonometric lookup tables for
  the mixed radix real float fft algorithm"
  :export nil
  :documentation
  "These functions prepare trigonometric lookup tables for an FFT of size n
  real float elements. The functions return a pointer to the newly allocated
  struct if no errors were detected, and a null pointer in the case of error.
  The length n is factorized into a product of subtransforms, and the factors
  and their trigonometric coefficients are stored in the wavetable. The
  trigonometric coefficients are computed using direct calls to sin and cos,
  for accuracy. Recursion relations could be used to compute the lookup table
  faster, but if an application performs many FFTs of the same length then
  computing the wavetable is a one-off overhead which does not affect the
  final throughput.

  The wavetable structure can be used repeatedly for any transform of the same
  length. The table is not modified by calls to any of the other FFT
  functions. The appropriate type of wavetable must be used for forward real
  or inverse half-complex transforms.")

(defmobject fft-real-workspace-double-float
    "gsl_fft_real_workspace" ((n :sizet))
  "Structure that holds the additional working space required for the
  intermediate steps of the mixed radix real fft algoritms"
  :export nil
  :documentation
  "This function allocates a workspace for a real transform of length n.")

(defmobject fft-real-workspace-single-float
    "gsl_fft_real_workspace_float" ((n :sizet))
  "Structure that holds the additional working space required for the
  intermediate steps of the mixed radix real float fft algoritms"
  :export nil
  :documentation
  "This function allocates a workspace for a real float transform of length
  n.")

;;;;****************************************************************************
;;;; Complex
;;;;****************************************************************************

;; /usr/include/gsl/gsl_fft_complex.h
;; /usr/include/gsl/gsl_fft_complex_float.h

(defmobject fft-complex-wavetable-double-float
    "gsl_fft_complex_wavetable" ((n :sizet))
  "structure that holds the factorization and trigonometric lookup tables for
  the mixed radix complex fft algorithm"
  :export nil
  :documentation
  "This function prepares a trigonometric lookup table for a complex FFT of
  length n. The function returns a pointer to the newly allocated
  gsl_fft_complex_wavetable if no errors were detected, and a null pointer in
  the case of error. The length n is factorized into a product of
  subtransforms, and the factors and their trigonometric coefficients are
  stored in the wavetable. The trigonometric coefficients are computed using
  direct calls to sin and cos, for accuracy. Recursion relations could be used
  to compute the lookup table faster, but if an application performs many FFTs
  of the same length then this computation is a one-off overhead which does
  not affect the final throughput.
  
  The wavetable structure can be used repeatedly for any transform of the same
  length. The table is not modified by calls to any of the other FFT
  functions. The same wavetable can be used for both forward and backward (or
  inverse) transforms of a given length.")

(defmobject fft-complex-wavetable-single-float
    "gsl_fft_complex_wavetable_float" ((n :sizet))
  "structure that holds the factorization and trigonometric lookup tables for
  the mixed radix complex float fft algorithm"
  :export nil
  :documentation
  "This function prepares a trigonometric lookup table for a complex float FFT
  of length n. The function returns a pointer to the newly allocated
  gsl_fft_complex_wavetable if no errors were detected, and a null pointer in
  the case of error. The length n is factorized into a product of
  subtransforms, and the factors and their trigonometric coefficients are
  stored in the wavetable. The trigonometric coefficients are computed using
  direct calls to sin and cos, for accuracy. Recursion relations could be used
  to compute the lookup table faster, but if an application performs many FFTs
  of the same length then this computation is a one-off overhead which does
  not affect the final throughput.
  
  The wavetable structure can be used repeatedly for any transform of the same
  length. The table is not modified by calls to any of the other FFT
  functions. The same wavetable can be used for both forward and backward (or
  inverse) transforms of a given length.")

(defmobject fft-complex-workspace-double-float
    "gsl_fft_complex_workspace" ((n :sizet))
  "Structure that holds the additional working space required for the
  intermediate steps of the mixed radix complex fft algoritms"
  :export nil
  :documentation
  "This function allocates a workspace for a complex transform of length n.")

(defmobject fft-complex-workspace-single-float
    "gsl_fft_complex_workspace_float" ((n :sizet))
  "Structure that holds the additional working space required for the
  intermediate steps of the mixed radix complex float fft algoritms"
  :export nil
  :documentation
  "This function allocates a workspace for a complex float transform of length
  n.")

;;;;****************************************************************************
;;;; Half complex
;;;;****************************************************************************

;; /usr/include/gsl/gsl_fft_halfcomplex.h
;; /usr/include/gsl/gsl_fft_halfcomplex_float.h

(defmobject fft-half-complex-wavetable-double-float
    "gsl_fft_halfcomplex_wavetable" ((n :sizet))
  "structure that holds the factorization and trigonometric lookup tables for
  the mixed radix halfcomplex fft algorithm"
  :export nil
  :documentation
  "These functions prepare trigonometric lookup tables for an FFT of size n
  real elements. The functions return a pointer to the newly allocated struct
  if no errors were detected, and a null pointer in the case of error. The
  length n is factorized into a product of subtransforms, and the factors and
  their trigonometric coefficients are stored in the wavetable. The
  trigonometric coefficients are computed using direct calls to sin and cos,
  for accuracy. Recursion relations could be used to compute the lookup table
  faster, but if an application performs many FFTs of the same length then
  computing the wavetable is a one-off overhead which does not affect the
  final throughput.

  The wavetable structure can be used repeatedly for any transform of the same
  length. The table is not modified by calls to any of the other FFT
  functions. The appropriate type of wavetable must be used for forward real
  or inverse half-complex transforms.")

(defmobject fft-half-complex-wavetable-single-float
    "gsl_fft_halfcomplex_wavetable_float" ((n :sizet))
  "structure that holds the factorization and trigonometric lookup tables for
  the mixed radix real float fft algorithm"
  :export nil
  :documentation
  "These functions prepare trigonometric lookup tables for an FFT of size n
  real float elements. The functions return a pointer to the newly allocated
  struct if no errors were detected, and a null pointer in the case of error.
  The length n is factorized into a product of subtransforms, and the factors
  and their trigonometric coefficients are stored in the wavetable. The
  trigonometric coefficients are computed using direct calls to sin and cos,
  for accuracy. Recursion relations could be used to compute the lookup table
  faster, but if an application performs many FFTs of the same length then
  computing the wavetable is a one-off overhead which does not affect the
  final throughput.

  The wavetable structure can be used repeatedly for any transform of the same
  length. The table is not modified by calls to any of the other FFT
  functions. The appropriate type of wavetable must be used for forward real
  or inverse half-complex transforms.")

;;;;****************************************************************************
;;;; General makers
;;;;****************************************************************************

;;; Generalised ways of making wavetables
(export 'make-fft-wavetable)
(defun make-fft-wavetable (element-type dimension &optional (half-complex nil))
  "Make a wavetable for an FFT of the given element type and length. T can be
  given as an optional third argument if the wavetable is meant for a Fourier
  transform on a half-complex vector."
  (cond ((eql element-type 'single-float)
         (if half-complex
           (make-fft-half-complex-wavetable-single-float dimension)
           (make-fft-real-wavetable-single-float dimension)))
        ((eql element-type 'double-float)
         (if half-complex
           (make-fft-half-complex-wavetable-double-float dimension)
           (make-fft-real-wavetable-double-float dimension)))
        ((equal element-type '(complex single-float))
         (make-fft-complex-wavetable-single-float dimension))
        ((equal element-type '(complex double-float))
         (make-fft-complex-wavetable-double-float dimension))))

;;; Generalised ways of making workspaces
(export 'make-fft-workspace)
(defun make-fft-workspace (element-type dimension)
  "Make a wavetable for an FFT of the given element type and length."
  (cond ((eql element-type 'single-float)
         (make-fft-real-workspace-single-float dimension))
        ((eql element-type 'double-float)
         (make-fft-real-workspace-double-float dimension))
        ((equal element-type '(complex single-float))
         (make-fft-complex-workspace-single-float dimension))
        ((equal element-type '(complex double-float))
         (make-fft-complex-workspace-double-float dimension))))

;;; An environment to allow more efficient FFTs of the same type and length
(export 'with-fourier-transform-environment)
(defmacro with-fourier-transform-environment
  ((wavetable workspace element-type dimension &optional (half-complex nil))
   &body body)
  "Create an environment where all FFTs will be performed on vectors of the
  same type and with the same length. This allows to calculculate and reuse the
  wavetable and workspace only once.

  The first and second arguments will be bound to the wavetable and workspace,
  the third argument is the element type of the vectors to be FFT'd and the
  fourth argument indicates the length of the vectors to which FFTs will be
  applied. Optionally, T can be given as a fifth argument if the element type
  of the vectors is real, but must be considered as half-complex."
  `(let ((,wavetable (make-fft-wavetable ,element-type ,dimension ,half-complex))
         (,workspace (make-fft-workspace ,element-type ,dimension)))
     ,@body))

