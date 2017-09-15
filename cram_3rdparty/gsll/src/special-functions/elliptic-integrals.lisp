;; Elliptic integrals
;; Liam Healy, Mon Mar 20 2006 - 21:50
;; Time-stamp: <2016-08-06 22:03:19EDT elliptic-integrals.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2011, 2016 Liam M. Healy
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
;;;; Legendre form of complete elliptic integrals
;;;;****************************************************************************

(defmfun elliptic-integral-K-complete (k &optional (mode :double))
  "gsl_sf_ellint_Kcomp_e" ((k :double) (mode sf-mode) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The complete elliptic integral of the first kind, K(k).  Note that
  Abramowitz & Stegun define this function in terms of the parameter m
  = k^2.")

(defmfun elliptic-integral-E-complete (k &optional (mode :double))
  "gsl_sf_ellint_Ecomp_e" ((k :double) (mode sf-mode) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The complete elliptic integral of the second kind, E(k).
   Note that Abramowitz & Stegun define this function in terms of the
   parameter m = k^2.")

;;;;****************************************************************************
;;;; Legendre form of incomplete elliptic integrals
;;;;****************************************************************************

(defmfun elliptic-integral-F (phi k &optional (mode :double))
  "gsl_sf_ellint_F_e" ((phi :double) (k :double) (mode sf-mode) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The incomplete elliptic integral of the first kind, F(phi,k).  Note
  that Abramowitz & Stegun define this function in terms of the
  parameter m = k^2.")

(defmfun elliptic-integral-E (phi k &optional (mode :double))
  "gsl_sf_ellint_E_e" ((phi :double) (k :double) (mode sf-mode) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The incomplete elliptic integral of the second kind, E(phi,k).  Note
  that Abramowitz & Stegun define this function in terms of the
  parameter m = k^2.")

(defmfun elliptic-integral-P (phi k n &optional (mode :double))
  "gsl_sf_ellint_P_e"
  ((phi :double) (k :double) (n :double) (mode sf-mode) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The incomplete elliptic integral of the third kind, P(phi,k,n).
  Note that Abramowitz & Stegun define this function in terms of the
  parameters m = k^2 and sin^2(alpha) = k^2, with the change of sign
  n to -n.")

#-gsl2 ; GSL v1.x
(defmfun elliptic-integral-D (phi k n &optional (mode :double))
  "gsl_sf_ellint_D_e"
  ((phi :double) (k :double) (n :double) (mode sf-mode) (ret (:pointer (:struct sf-result))))
  :documentation			; GSL texi
  "The incomplete elliptic integral D(phi,k,n) which is defined through the Carlson form RD(x,y,z) by the relation D(phi,k,n) = RD (1-sin^2(phi), 1-k^2 sin^2(phi), 1).")

#+gsl2 ; GSL v2.x
(defmfun elliptic-integral-D (phi k &optional (mode :double))
  "gsl_sf_ellint_D_e"
  ((phi :double) (k :double) (mode sf-mode) (ret (:pointer (:struct sf-result))))
  :documentation			; GSL texi
  "The incomplete elliptic integral D(phi,k) which is defined through the Carlson form RD(x,y,z) by the relation D(phi,k) = 1/3 (\sin \phi)^3 RD (1-\sin^2(\phi), 1-k^2 \sin^2(\phi), 1).")

;;;;****************************************************************************
;;;; Carlson forms
;;;;****************************************************************************

(defmfun elliptic-integral-RC (x y &optional (mode :double))
  "gsl_sf_ellint_RC_e" ((x :double) (y :double) (mode sf-mode) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The incomplete elliptic integral RC(x,y).")

(defmfun elliptic-integral-RD (x y z &optional (mode :double))
  "gsl_sf_ellint_RD_e"
  ((x :double) (y :double) (z :double) (mode sf-mode) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The incomplete elliptic integral RD(x,y,z).")

(defmfun elliptic-integral-RF (x y z &optional (mode :double))
  "gsl_sf_ellint_RF_e"
  ((x :double) (y :double) (z :double) (mode sf-mode) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The incomplete elliptic integral RF(x,y,z).")

(defmfun elliptic-integral-RJ (x y z p &optional (mode :double))
  "gsl_sf_ellint_RJ_e"
  ((x :double) (y :double) (z :double) (p :double) (mode sf-mode)
	       (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The incomplete elliptic integral RJ(x,y,z,p).")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test elliptic-integrals
  (elliptic-integral-K-complete 0.0d0)
  (elliptic-integral-E-complete 0.0d0)
  (elliptic-integral-F -0.5d0 2.0d0)
  (elliptic-integral-E -0.5d0 2.0d0)
  (elliptic-integral-P -0.5d0 2.0d0 1.0d0)
  (elliptic-integral-D -0.5d0 2.0d0 1.0d0)
  (elliptic-integral-RC 2.0d0 1.0d0)
  (elliptic-integral-RD 2.0d0 1.0d0 1.0d0)
  (elliptic-integral-RF 2.0d0 1.0d0 1.0d0)
  (elliptic-integral-RJ 2.0d0 1.0d0 1.0d0 2.0d0))

