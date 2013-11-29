;; Exponential functions
;; Liam Healy, Tue Mar 21 2006 - 17:05
;; Time-stamp: <2010-05-31 23:32:15EDT exponential-functions.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009 Liam M. Healy
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
;;;; Exponential Functions
;;;;****************************************************************************

(defmfun gsl-exp (x)
  "gsl_sf_exp_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The exponential function.")

(defmfun exp-scaled (x)
  "gsl_sf_exp_e10_e" ((x :double) (ret sf-result-e10))
  :documentation			; FDL
  "The exponential function scaled. This function may be useful if the value
   of exp(x) would overflow the numeric range of double.")

(defmfun exp-mult (x y)
  "gsl_sf_exp_mult_e" ((x :double) (y :double) (ret sf-result))
  :documentation			; FDL
  "Exponentiate x and multiply by the factor y to
   return the product y \exp(x).")

(defmfun exp-mult-scaled (x y)
  "gsl_sf_exp_mult_e10_e" ((x :double) (y :double) (ret sf-result-e10))
  :documentation			; FDL
  "The product y \exp(x) with extended numeric range.")

;;;;****************************************************************************
;;;; Relative Exponential Functions
;;;;****************************************************************************

(defmfun expm1 (x)
  "gsl_sf_expm1_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "\exp(x)-1 using an algorithm that is accurate for small x.")

(defmfun exprel (x)
  "gsl_sf_exprel_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "(\exp(x)-1)/x using an algorithm that is accurate for small x.
  For small x the algorithm is based on the expansion
  (\exp(x)-1)/x = 1 + x/2 + x^2/(2*3) + x^3/(2*3*4) + ...")

(defmfun exprel-2 (x)
  "gsl_sf_exprel_2_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "2(\exp(x)-1-x)/x^2 using an algorithm that is accurate for small
   x.  For small x the algorithm is based on the expansion
   2(\exp(x)-1-x)/x^2 = 1 + x/3 + x^2/(3*4) + x^3/(3*4*5) + ...")

(defmfun exprel-n (n x)
  "gsl_sf_exprel_n_e" ((n :int) (x :double) (ret sf-result))
  :documentation			; FDL
  "N-relative exponential, which is the n-th generalization
   of the functions #'exprel and #'exprel-2.")

;;;;****************************************************************************
;;;; Exponentiation With Error Estimate
;;;;****************************************************************************

(defmfun exp-err (x dx)
  "gsl_sf_exp_err_e" ((x :double) (dx :double) (ret sf-result))
  :documentation			; FDL
  "Exponentiate x with an associated absolute error dx.")

(defmfun exp-err-scaled (x dx)
  "gsl_sf_exp_err_e10_e"
  ((x :double) (dx :double) (ret sf-result-e10))
  :documentation			; FDL
  "Exponentiate x with an associated absolute error dx
  and with extended numeric range.")

(defmfun exp-mult-err (x dx y dy)
  "gsl_sf_exp_mult_err_e"
  ((x :double) (dx :double) (y :double) (dy :double) (ret sf-result))
  :documentation			; FDL
  "The product y \exp(x) for the quantities x, y
   with associated absolute errors dx, dy.")

(defmfun exp-mult-err-scaled (x dx y dy)
  "gsl_sf_exp_mult_err_e10_e"
  ((x :double) (dx :double) (y :double) (dy :double) (ret sf-result-e10))
  :documentation			; FDL
  "The product y \exp(x) for the quantities x, y
   with associated absolute errors dx, dy and with
   extended numeric range.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test exponential-functions
  (gsl-exp 3.0d0)
  (exp-scaled 2000.0d0)
  (exp-mult 101.0d0 5.0d0)
  (exp-mult-scaled 555.0d0 101.0d0)
  (expm1 0.0001d0)
  (exprel 0.0001d0)
  (exprel-2 0.001d0)
  (exprel-n 3 0.001d0)
  (exp-err 3.0d0 0.001d0)
  (exp-mult-err 3.0d0 0.001d0 23.0d0 0.001d0))
