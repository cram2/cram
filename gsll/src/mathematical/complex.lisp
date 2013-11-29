;; Functions of complex numbers
;; Liam Healy 2009-01-13 21:19:38EST complex.lisp
;; Time-stamp: <2010-06-27 18:03:16EDT complex.lisp>
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

(in-package :gsl)

;; /usr/include/gsl/gsl_complex_math.h

;;;;****************************************************************************
;;; Properties
;;;;****************************************************************************

(defmfun argument (number)
  "gsl_complex_arg"
  ((number grid:complex-double-c))
  :c-return :double
  :documentation
  "The angle of the complex number.")

(defmfun modulus (number)
  "gsl_complex_abs"
  ((number grid:complex-double-c))
  :c-return :double
  :documentation "The magnitude, or modulus of the complex number.")

(defmfun modulus2 (number)
  "gsl_complex_abs2"
  ((number grid:complex-double-c))
  :c-return :double
  :documentation "The magnitude squared of the complex number.")

(defmfun log-modulus (number)
  "gsl_complex_logabs"
  ((number grid:complex-double-c))
  :c-return :double
  :documentation "The logarithm of the magnitude of the complex number.")

;;;;****************************************************************************
;;; Complex arithmetic
;;;;****************************************************************************

;;; The GSL functions provided duplicate what CL has built-in.
;;; These should have better thought-out names, but it's not clear
;;; that this will ever be used, because CL already does this.

(defmfun cx-add (c1 c2)
  "gsl_complex_add"
  ((c1 grid:complex-double-c) (c2 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-sub (c1 c2)
  "gsl_complex_sub"
  ((c1 grid:complex-double-c) (c2 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-mul (c1 c2)
  "gsl_complex_mul"
  ((c1 grid:complex-double-c) (c2 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-div (c1 c2)
  "gsl_complex_div"
  ((c1 grid:complex-double-c) (c2 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-add-real (c1 num)
  "gsl_complex_add_real"
  ((c1 grid:complex-double-c) (num :double))
  :c-return grid:complex-double-c)

(defmfun cx-sub-real (c1 num)
  "gsl_complex_sub_real"
  ((c1 grid:complex-double-c) (num :double))
  :c-return grid:complex-double-c)

(defmfun cx-mul-real (c1 num)
  "gsl_complex_mul_real"
  ((c1 grid:complex-double-c) (num :double))
  :c-return grid:complex-double-c)

(defmfun cx-div-real (c1 num)
  "gsl_complex_div_real"
  ((c1 grid:complex-double-c) (num :double))
  :c-return grid:complex-double-c)

(defmfun cx-add-imag (c1 num)
  "gsl_complex_add_imag"
  ((c1 grid:complex-double-c) (num :double))
  :c-return grid:complex-double-c)

(defmfun cx-sub-imag (c1 num)
  "gsl_complex_sub_imag"
  ((c1 grid:complex-double-c) (num :double))
  :c-return grid:complex-double-c)

(defmfun cx-mul-imag (c1 num)
  "gsl_complex_mul_imag"
  ((c1 grid:complex-double-c) (num :double))
  :c-return grid:complex-double-c)

(defmfun cx-div-imag (c1 num)
  "gsl_complex_div_imag"
  ((c1 grid:complex-double-c) (num :double))
  :c-return grid:complex-double-c)

(defmfun cx-conjugate (c1)
  "gsl_complex_conjugate"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-inverse (c1)
  "gsl_complex_inverse"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-negative (c1)
  "gsl_complex_negative"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

;;;;****************************************************************************
;;; Complex functions
;;;;****************************************************************************

;;; The GSL functions provided duplicate what CL has built-in.
;;; These should have better thought-out names/interfaces, but it's not clear
;;; that this will ever be used, because CL already does this.

(defmfun cx-sqrt (c1)
  "gsl_complex_sqrt"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-sqrt-real (num)
  "gsl_complex_sqrt_real"
  ((num :double))
  :c-return grid:complex-double-c)

(defmfun cx-expt (c1 c2)
  "gsl_complex_pow"
  ((c1 grid:complex-double-c) (c2 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-expt-real (c1 num)
  "gsl_complex_pow_real"
  ((c1 grid:complex-double-c) (num :double))
  :c-return grid:complex-double-c)

(defmfun cx-exp (c1)
  "gsl_complex_exp"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-log (c1)
  "gsl_complex_log"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-log10 (c1)
  "gsl_complex_log10"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-logb (c1 c2)
  "gsl_complex_log_b"
  ((c1 grid:complex-double-c) (c2 grid:complex-double-c))
  :c-return grid:complex-double-c)

;;;;****************************************************************************
;;; Trigonometric functions
;;;;****************************************************************************

;;; The GSL functions provided duplicate what CL has built-in.
;;; These should have better thought-out names/interfaces, but it's not clear
;;; that this will ever be used, because CL already does this.

(defmfun cx-sin (c1)
  "gsl_complex_sin"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-cos (c1)
  "gsl_complex_cos"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-sec (c1)
  "gsl_complex_sec"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-csc (c1)
  "gsl_complex_csc"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-tan (c1)
  "gsl_complex_tan"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-cot (c1)
  "gsl_complex_cot"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

;;;;****************************************************************************
;;; Inverse trigonometric functions
;;;;****************************************************************************

;;; The GSL functions provided duplicate what CL has built-in.
;;; These should have better thought-out names/interfaces, but it's not clear
;;; that this will ever be used, because CL already does this.

(defmfun cx-arcsin (c1)
  "gsl_complex_arcsin"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-arcsin-real (num)
  "gsl_complex_arcsin_real"
  ((num :double))
  :c-return grid:complex-double-c)

(defmfun cx-arccos (c1)
  "gsl_complex_arccos"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-arccos-real (num)
  "gsl_complex_arccos_real"
  ((num :double))
  :c-return grid:complex-double-c)

(defmfun cx-arcsec (c1)
  "gsl_complex_arcsec"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-arcsec-real (num)
  "gsl_complex_arcsec_real"
  ((num :double))
  :c-return grid:complex-double-c)

(defmfun cx-arccsc (c1)
  "gsl_complex_arccsc"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-arccsc-real (num)
  "gsl_complex_arccsc_real"
  ((num :double))
  :c-return grid:complex-double-c)

(defmfun cx-arctan (c1)
  "gsl_complex_arctan"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-arccot (c1)
  "gsl_complex_arccot"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

;;;;****************************************************************************
;;; Hyperbolic functions
;;;;****************************************************************************

(defmfun cx-sinh (c1)
  "gsl_complex_sinh"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-cosh (c1)
  "gsl_complex_cosh"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-sech (c1)
  "gsl_complex_sech"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-csch (c1)
  "gsl_complex_csch"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-tanh (c1)
  "gsl_complex_tanh"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-coth (c1)
  "gsl_complex_coth"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

;;;;****************************************************************************
;;; Inverse trigonometric functions
;;;;****************************************************************************

(defmfun cx-arcsinh (c1)
  "gsl_complex_arcsinh"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-arccosh (c1)
  "gsl_complex_arccosh"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-arccosh-real (num)
  "gsl_complex_arccosh_real"
  ((num :double))
  :c-return grid:complex-double-c)

(defmfun cx-arcsech (c1)
  "gsl_complex_arcsech"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-arccsch (c1)
  "gsl_complex_arccsch"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-arctanh (c1)
  "gsl_complex_arctanh"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)

(defmfun cx-arctanh-real (num)
  "gsl_complex_arctanh_real"
  ((num :double))
  :c-return grid:complex-double-c)

(defmfun cx-arccoth (c1)
  "gsl_complex_arccoth"
  ((c1 grid:complex-double-c))
  :c-return grid:complex-double-c)
