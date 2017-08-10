;; Trigonometry
;; Liam Healy, Thu May  4 2006 - 22:58
;; Time-stamp: <2011-10-29 23:44:51EDT trigonometry.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2011 Liam M. Healy
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

;;;;****************************************************************************
;;;; Circular Trigonometric Functions
;;;;****************************************************************************

(defgeneric gsl-sin (x)
  (:documentation "The sine function sin(x)."))

(defgeneric gsl-cos (x)
  (:documentation "The cosine function cos(x)."))

(defmfun gsl-sin ((x float))
  "gsl_sf_sin_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :definition :method
  :export t)

(defmfun gsl-sin ((x complex))
  "gsl_sf_complex_sin_e"
  (((realpart x) :double) ((imagpart x) :double)
   (re-ret (:pointer (:struct sf-result)))
   (im-ret (:pointer (:struct sf-result))))
  :definition :method
  :return ((complex-with-error re-ret im-ret)))

(defmfun gsl-cos ((x float))
  "gsl_sf_cos_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :definition :method
  :export t)

(defmfun gsl-cos ((x complex))
  "gsl_sf_complex_cos_e"
  (((realpart x) :double) ((imagpart x) :double)
   (re-ret (:pointer (:struct sf-result)))
   (im-ret (:pointer (:struct sf-result))))
  :definition :method 
  :return ((complex-with-error re-ret im-ret)))

(defmfun hypotenuse (x y)
  "gsl_sf_hypot_e"
  ((x :double) (y :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The hypotenuse function sqrt{x^2 + y^2}.")

(defmfun sinc (x)
  "gsl_sf_sinc_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "sinc(x) = sin(pi x) / (pi x)}")

(defmfun log-sin (x)
  "gsl_sf_complex_logsin_e"
  (((realpart x) :double) ((imagpart x) :double)
   (re-ret (:pointer (:struct sf-result)))
   (im-ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "This function computes the logarithm of the complex sine,
  \log(\sin(z_r + i z_i)) storing the real and imaginary parts in
  szr, szi."
  :return ((complex-with-error re-ret im-ret)))

;;;;****************************************************************************
;;;; Hyperbolic Trigonometric Functions
;;;;****************************************************************************

(defmfun log-sinh (x)
  "gsl_sf_lnsinh_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "Logarithm of sinh function, special functions
  These routines compute log(\sinh(x)) for x > 0.")

(defmfun log-cosh (x)
  "gsl_sf_lncosh_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "Logarithm of cosh function, special functions
   These routines compute log(cosh(x)) for any x.")

;;;;****************************************************************************
;;;; Conversion Functions
;;;;****************************************************************************

(defmfun polar-to-rectangular (r theta)
  "gsl_sf_polar_to_rect"
  ((r :double) (theta :double) (x (:pointer (:struct sf-result)))
	       (y (:pointer (:struct sf-result))))
  :return ((values-with-errors x y))
  :documentation			; FDL
  "Convert the polar coordinates (r, theta) to
  rectilinear coordinates (x, y), x = r\cos(\theta), y = r\sin(\theta).")

(defmfun rectangular-to-polar (x y)
  "gsl_sf_rect_to_polar"
  ((x :double) (y :double) (r (:pointer (:struct sf-result)))
	       (theta (:pointer (:struct sf-result))))
  :return ((values-with-errors r theta))
  :documentation			; FDL
  "Convert the rectilinear coordinates (x, y) to
  polar coordinates (r, theta), such that x =
  r cos(theta)}, y = r sin(theta).  The argument theta
  lies in the range [-\pi, \pi].")

;;;;****************************************************************************
;;;; Restriction Functions
;;;;****************************************************************************

(defmfun restrict-symmetric (theta)
  "gsl_sf_angle_restrict_symm" ((theta :double))
  :c-return :double
  :documentation			; FDL
  "Force the angle theta to lie in the range (-\pi,\pi].")

(defmfun restrict-positive (theta)
  "gsl_sf_angle_restrict_pos" ((theta :double))
  :c-return :double
  :documentation			; FDL
  "Force the angle theta to lie in the range [0,2\pi).")

;;;;****************************************************************************
;;;; Trigonometric Functions With Error Estimates
;;;;****************************************************************************

(defmfun sin-err (x dx)
  "gsl_sf_sin_err_e" ((x :double) (dx :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "Compute the sine of an angle x with
   an associated absolute error dx, sin(x \pm dx).")

(defmfun cos-err (x dx)
  "gsl_sf_cos_err_e" ((x :double) (dx :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The cosine of an angle x with an associated
  absolute error dx, cos(x \pm dx).")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test trigonometry
  (gsl-sin 1.0d0)
  (gsl-sin #C(1.0d0 1.0d0))
  (gsl-cos 1.0d0)
  (gsl-cos #C(1.0d0 1.0d0))
  (hypotenuse 1.0d0 2.0d0)
  (sinc 0.5d0)
  (log-sin #C(1.0d0 1.0d0))
  (log-sinh 0.5d0)
  (log-cosh 0.5d0)
  (polar-to-rectangular 2.0d0 1.0d0)
  (rectangular-to-polar 2.0d0 1.0d0)
  (restrict-symmetric 5.0d0)
  (restrict-positive -1.0d0)
  (sin-err 0.5d0 0.01d0)
  (cos-err 0.5d0 0.01d0))

