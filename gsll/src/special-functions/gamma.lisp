;; Gamma functions
;; Liam Healy, Thu Apr 27 2006 - 22:06
;; Time-stamp: <2010-06-27 18:03:15EDT gamma.lisp>
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

;;; Need to handle incoming gsl-complex numbers correctly for log-gamma-complex.
;;; Should functions returning sf-result and something else return the
;;; error at the end?

;;; /usr/include/gsl/gsl_sf_gamma.h

;;;;****************************************************************************
;;;; Gamma functions
;;;;****************************************************************************

(defconstant +gamma-xmax+ 171.0d0)

(defmfun gamma (x)
  "gsl_sf_gamma_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The Gamma function Gamma(x), subject to x
   not being a negative integer.  The function is computed using the real
   Lanczos method. The maximum value of x such that
   Gamma(x) is not considered an overflow is given by +gamma-xmax+.")

(defmfun log-gamma (x)
  "gsl_sf_lngamma_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The logarithm of the Gamma function,
   log(Gamma(x)), subject to x not a being negative
   integer.  For x<0 the real part of log(Gamma(x)) is
   returned, which is equivalent to log(|Gamma(x)|).  The function
   is computed using the real Lanczos method.")

(defmfun log-gamma-sign (x)
  "gsl_sf_lngamma_sgn_e"
  ((x :double) (ret sf-result) (sign (:pointer :double)))
  :return ((val ret) (grid:dcref sign) (err ret))
  :documentation			; FDL
  "Compute the sign of the gamma function and the logarithm of
  its magnitude, subject to x not being a negative integer.  The
  function is computed using the real Lanczos method.  The value of the
  gamma function can be reconstructed using the relation Gamma(x) =
  sgn * exp(resultlg)}.")

(defmfun gamma* (x)
  "gsl_sf_gammastar_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The regulated Gamma Function Gamma^*(x)
  for x > 0, given by
  Gamma^*(x) = Gamma(x)/(sqrt{2\pi} x^{(x-1/2)} \exp(-x))
            = (1 + {1 \over 12x} + ...)
  for x to infinity.")

(defmfun 1/gamma (x)
  "gsl_sf_gammainv_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The reciprocal of the gamma function,
  1/\Gamma(x) using the real Lanczos method.")

(defmfun log-gamma-complex (z)
  "gsl_sf_lngamma_complex_e"
  (((realpart z) :double) ((imagpart z) :double)
   (lnr sf-result) (arg sf-result))
  :documentation			; FDL
  "Compute log(Gamma(z)) for complex z=z_r+i z_i
  and z not a negative integer, using the complex Lanczos
  method.  The returned parameters are lnr = log|\Gamma(z)| and
  arg = arg(Gamma(z)) in (-pi,pi].  Note that the phase
  part (arg) is not well-determined when |z| is very large,
  due to inevitable roundoff in restricting to (-\pi,\pi].  This
  will result in a :ELOSS error when it occurs.  The absolute
  value part (lnr), however, never suffers from loss of precision."
  :return
  ((val lnr) (val arg) (err lnr) (err arg)))

(defmfun taylor-coefficient (n x)
  "gsl_sf_taylorcoeff_e" ((n :int) (x :double) (ret sf-result))
  :documentation			; FDL
  "Compute the Taylor coefficient x^n / n! for x >= 0, n >= 0.")

(defmfun factorial (n)
  "gsl_sf_fact_e" ((n sizet) (ret sf-result))
  :documentation			; FDL
  "The factorial n!, related to the Gamma function by n! = \Gamma(n+1).")

(defmfun double-factorial (n)
  "gsl_sf_doublefact_e" ((n sizet) (ret sf-result))
  :documentation			; FDL
  "The double factorial n!! = n(n-2)(n-4) \dots.")

(defmfun log-factorial (n)
  "gsl_sf_lnfact_e" ((n sizet) (ret sf-result))
  :documentation			; FDL
  "The logarithm of the factorial of n, log(n!).
  The algorithm is faster than computing
  ln(Gamma(n+1)) via #'log-gamma for n < 170, but defers for larger n.")

(defmfun log-double-factorial (n)
  "gsl_sf_lndoublefact_e" ((n sizet) (ret sf-result))
  :documentation			; FDL
  "Compute the logarithm of the double factorial of n, log(n!!).")

(defmfun choose (n m)
  "gsl_sf_choose_e" ((n sizet) (m sizet) (ret sf-result))
  :documentation			; FDL
  "The combinatorial factor (n choose m) = n!/(m!(n-m)!).")

(defmfun log-choose (n m)
  "gsl_sf_lnchoose_e" ((n sizet) (m sizet) (ret sf-result))
  :documentation			; FDL
  "The logarithm of (n choose m).  This is
  equivalent to the sum log(n!) - log(m!) - log((n-m)!).")

(defmfun pochammer (a x)
  "gsl_sf_poch_e"  ((a :double) (x :double) (ret sf-result))
  :documentation			; FDL
  "The Pochhammer symbol (a)_x := Gamma(a +
   x)/Gamma(a), subject to a and a+x not being negative
   integers. The Pochhammer symbol is also known as the Apell symbol and
   sometimes written as (a,x).")

(defmfun log-pochammer (a x)
  "gsl_sf_lnpoch_e" ((a :double) (x :double) (ret sf-result))
  :documentation			; FDL
  "The logarithm of the Pochhammer symbol,
  log((a)_x) = log(Gamma(a + x)/Gamma(a)) for a > 0, a+x > 0.")

(defmfun log-pochammer-sign (a x)
  "gsl_sf_lnpoch_sgn_e"
  ((a :double) (x :double) (ret sf-result) (sign (:pointer :double)))
  :documentation			; FDL
  "The logarithm of the Pochhammer symbol and its sign.
  The computed parameters are result =
  log(|(a)_x|) and sgn = sgn((a)_x) where (a)_x :=
  Gamma(a + x)/Gamma(a), subject to a, a+x not being negative integers."
  :return ((val ret) (grid:dcref sign) (err ret)))

(defmfun relative-pochammer (a x)
  "gsl_sf_pochrel_e" ((a :double) (x :double) (ret sf-result))
  :documentation			; FDL
  "The relative Pochhammer symbol ((a)_x -
  1)/x where (a)_x := Gamma(a + x)/Gamma(a)}.")

(defmfun incomplete-gamma (a x)
  "gsl_sf_gamma_inc_Q_e" ((a :double) (x :double) (ret sf-result))
  :documentation			; FDL
  "The normalized incomplete Gamma Function
  Q(a,x) = 1/Gamma(a) \int_x^\infty dt t^{a-1} \exp(-t) for a > 0, x >= 0.")

(defmfun complementary-incomplete-gamma (a x)
  "gsl_sf_gamma_inc_P_e" ((a :double) (x :double) (ret sf-result))
  :documentation			; FDL
  "The complementary normalized incomplete Gamma Function
  P(a,x) = 1/\Gamma(a) \int_0^x dt t^{a-1} \exp(-t)}
  for a > 0, x >= 0.  Note that Abramowitz & Stegun
  call P(a,x) the incomplete gamma function (section 6.5).")

(defmfun nonnormalized-incomplete-gamma (a x)
  "gsl_sf_gamma_inc_e" ((a :double) (x :double) (ret sf-result))
  :documentation			; FDL
  "The incomplete Gamma Function
   Gamma(a,x), without the normalization factor
   included in the previously defined functions:
   Gamma(a,x) = \int_x^\infty dt t^{a-1} \exp(-t)
   for a real and x >= 0.")

(defmfun beta (a b)
  "gsl_sf_beta_e" ((a :double) (b :double) (ret sf-result))
  :documentation			; FDL
  "The Beta Function, B(a,b) = Gamma(a)Gamma(b)/Gamma(a+b)} for a > 0, b > 0.")

(defmfun log-beta (a b)
  "gsl_sf_lnbeta_e" ((a :double) (b :double) (ret sf-result))
  :documentation			; FDL
  "The logarithm of the Beta Function, log(B(a,b)) for a > 0, b > 0.")

(defmfun incomplete-beta (a b x)
  "gsl_sf_beta_inc_e"
  ((a :double) (b :double) (x :double) (ret sf-result))
  :documentation			; FDL
  "The normalized incomplete Beta function
   B_x(a,b)/B(a,b) where
   B_x(a,b) = \int_0^x t^{a-1} (1-t)^{b-1} dt
   for a > 0, b > 0, and 0 <= x <= 1.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test gamma
  (gamma -1.0d0)
  (gamma 6.0d0)
  (log-gamma -100.0d0)
  (log-gamma 100.0d0)
  (log-gamma-sign 100.0d0)
  (gamma* 24.0d0)
  (1/gamma 8.0d0)
  (log-gamma-complex #C(10.0d0 10.0d0))
  (taylor-coefficient 12 3.0d0)
  (factorial 12)
  (double-factorial 12)
  (log-factorial 199)
  (log-double-factorial 199)
  (choose 8 3)
  (choose 3 8)
  (log-choose 67 12)
  (pochammer 3.0d0 2.0d0)
  (log-pochammer 2.0d0 199.0d0)
  (log-pochammer-sign 2.0d0 199.0d0)
  (relative-pochammer 2.0d0 9.0d0)
  (incomplete-gamma 2.0d0 2.0d0)
  (complementary-incomplete-gamma 2.0d0 2.0d0)
  (nonnormalized-incomplete-gamma 2.0d0 2.0d0)
  (beta 5.50d0 1.0d0)
  (log-beta 5.5d0 1.0d0)
  (incomplete-beta 1.0d0 1.50d0 0.50d0))
