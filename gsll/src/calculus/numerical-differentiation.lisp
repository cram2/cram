;; Numerical differentiation.                
;; Liam Healy Mon Nov 12 2007 - 22:07
;; Time-stamp: <2009-12-27 09:42:10EST numerical-differentiation.lisp>
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

;;; /usr/include/gsl/gsl_diff.h

(defmfun central-derivative (function x step)
  "gsl_deriv_central"
  ((callback :pointer) (x :double) (step :double)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback fnstruct nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Compute the numerical derivative of the function
   at the point x using an adaptive central difference algorithm with
   a step-size of step.   The derivative and an
   estimate of its absolute error is returned.

   The initial value of step is used to estimate an optimal step-size,
   based on the scaling of the truncation error and round-off error in the
   derivative calculation.  The derivative is computed using a 5-point rule
   for equally spaced abscissae at x-step, x-step/2, x,
   x+step/2, x, with an error estimate taken from the difference
   between the 5-point rule and the corresponding 3-point rule x-step,
   x, x+step.  Note that the value of the function at x
   does not contribute to the derivative calculation, so only 4-points are
   actually used.")

(defmfun forward-derivative (function x step)
  "gsl_deriv_forward"
  ((callback :pointer) (x :double) (step :double)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback fnstruct nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Compute the numerical derivative of the function
   at the point x using an adaptive forward difference algorithm with
   a step-size of step.  The function is evaluated only at points greater
   than x and never at x itself.  The derivative is returned in
   result and an estimate of its absolute error is returned as the
   second value.  This function should be used if f(x) has a
   discontinuity at x, or is undefined for values less than x.

   The initial value of step is used to estimate an optimal step-size,
   based on the scaling of the truncation error and round-off error in
   the derivative calculation.  The derivative at x is computed
   using an ``open'' 4-point rule for equally spaced abscissae at
   x+step/4, x+step/2, x+3step/4, x+step,
   with an error estimate taken from the difference between the 4-point
   rule and the corresponding 2-point rule x+step/2,
   x+step.")

(defmfun backward-derivative (function x step)
  "gsl_deriv_backward"
  ((callback :pointer) (x :double) (step :double)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback fnstruct nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Compute the numerical derivative of the function at the point x
   using an adaptive backward difference algorithm with a step-size of
   step. The function is evaluated only at points less than x, and never
   at x itself.  The derivative is returned in result and an estimate of
   its absolute error is returned as the second value.  This function
   should be used if f(x) has a discontinuity at x, or is undefined for
   values greater than x.  This function is equivalent to calling
   #'forward-derivative with a negative step-size.")

;;;; Examples and unit test

;;; Examples from gsl-1.11/deriv/test.c

(defun deriv-f1-d (x) (exp x))
(defun deriv-f2 (x) (if (not (minusp x)) (expt x 3/2) 0.0d0))
(defun deriv-f2-d (x) (if (not (minusp x)) (* 3/2 (sqrt x)) 0.0d0))
(defun deriv-f3 (x) (if (not (zerop x)) (sin (/ x)) 0.0d0))
(defun deriv-f3-d (x) (if (not (zerop x)) (/ (- (cos (/ x))) (expt x 2)) 0.0d0))
(defun deriv-f4 (x) (exp (- (expt x 2))))
(defun deriv-f4-d (x) (* -2 x (exp (- (expt x 2)))))
(defun deriv-f5 (x) (expt x 2))
(defun deriv-f5-d (x) (* 2 x))
(defun deriv-f6-d (x) (- (expt x -2)))

(save-test numerical-differentiation
 (central-derivative 'exp 1.0d0 1.0d-4)
 (forward-derivative 'exp 1.0d0 1.0d-4)
 (backward-derivative 'exp 1.0d0 1.0d-4)
 (central-derivative 'deriv-f2 0.1d0 1.0d-4)
 (forward-derivative 'deriv-f2 0.1d0 1.0d-4)
 (backward-derivative 'deriv-f2 0.1d0 1.0d-4)
 (central-derivative 'deriv-f3 0.45d0 1.0d-4)
 (forward-derivative 'deriv-f3 0.45d0 1.0d-4)
 (backward-derivative 'deriv-f3 0.45d0 1.0d-4)
 (central-derivative 'deriv-f4 0.5d0 1.0d-4)
 (forward-derivative 'deriv-f4 0.5d0 1.0d-4)
 (backward-derivative 'deriv-f4 0.5d0 1.0d-4)
 (central-derivative 'deriv-f5 0.0d0 1.0d-4)
 (forward-derivative 'deriv-f5 0.0d0 1.0d-4)
 (backward-derivative 'deriv-f5 0.0d0 1.0d-4)
 (central-derivative '/ 10.0d0 1.0d-4)
 (forward-derivative '/ 10.0d0 1.0d-4)
 (backward-derivative '/ 10.0d0 1.0d-4))
