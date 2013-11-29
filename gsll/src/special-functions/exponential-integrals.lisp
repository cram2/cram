;; Exponential integrals
;; Liam Healy, Tue Mar 21 2006 - 17:37
;; Time-stamp: <2009-12-27 10:10:03EST exponential-integrals.lisp>
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
;;;; Exponential Integral
;;;;****************************************************************************

(defmfun exponential-integral-E1 (x)
  "gsl_sf_expint_E1_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The exponential integral
   E_1(x)}, E_1(x) := \Re \int_1^\infty dt \exp(-xt)/t..")

(defmfun exponential-integral-E2 (x)
  "gsl_sf_expint_E2_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The second-order exponential integral
   E_2(x)}, E_2(x) := \Re \int_1^\infty dt \exp(-xt)/t^2.")

(defmfun exponential-integral-En (n x)
  "gsl_sf_expint_En_e" ((n :int) (x :double) (ret sf-result))
  :gsl-version (1 10)
  :documentation			; FDL
  "The exponential integral E_n(x) of order n,
          E_n(x) := \Re \int_1^\infty dt \exp(-xt)/t^n.")


;;;;****************************************************************************
;;;; Ei
;;;;****************************************************************************

(defmfun exponential-integral-Ei (x)
    "gsl_sf_expint_Ei_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The exponential integral Ei(x),
   Ei(x) := - PV\left(\int_{-x}^\infty dt \exp(-t)/t\right).")

;;;;****************************************************************************
;;;; Hyperbolic Integrals
;;;;****************************************************************************

(defmfun Shi (x)
  "gsl_sf_Shi_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The integral Shi(x) = \int_0^x dt \sinh(t)/t.")

(defmfun Chi (x)
  "gsl_sf_Chi_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The integral
   Chi(x) := \Re[ \gamma_E + \log(x) + \int_0^x dt (\cosh[t]-1)/t],
   where \gamma_E} is the Euler constant.")

;;;;****************************************************************************
;;;; Ei-3
;;;;****************************************************************************

(defmfun exponential-integral-3 (x)
  "gsl_sf_expint_3_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The third-order exponential integral Ei_3(x) = \int_0^xdt \exp(-t^3)
  for x >= 0.")

;;;;****************************************************************************
;;;; Trigonometric Integrals
;;;;****************************************************************************

(defmfun Si (x)
  "gsl_sf_Si_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The Sine integral Si(x) = \int_0^x dt \sin(t)/t.")

(defmfun Ci (x)
  "gsl_sf_Ci_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The Cosine integral Ci(x) = -\int_x^\infty dt \cos(t)/t
   for x > 0.")

;;;;****************************************************************************
;;;; Trigonometric Integrals
;;;;****************************************************************************

(defmfun atanint (x)
  "gsl_sf_atanint_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The Arctangent integral, which is defined as
   AtanInt(x) = \int_0^x dt \arctan(t)/t.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test exponential-integrals
  (exponential-integral-E1 0.0d0)
  (exponential-integral-E1 1.0d0)
  (exponential-integral-Ei 2.0d0)
  (exponential-integral-En 2 1.0d0)
  (Shi 1.25d0)
  (Chi 1.25d0)
  (exponential-integral-3 1.25d0)
  (si 1.25d0)
  (ci 1.25d0)
  (atanint 1.25d0))
