;; Error functions
;; Liam Healy, Mon Mar 20 2006 - 22:31
;; Time-stamp: <2011-10-29 23:36:27EDT error-functions.lisp>
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

(defmfun erf (x)
  "gsl_sf_erf_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The error function erf(x), where
  erf(x) = (2/\sqrt(\pi)) \int_0^x dt \exp(-t^2).")

(defmfun erfc (x)
  "gsl_sf_erfc_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The complementary error function 
  erfc(x) = 1 - erf(x) = (2/\sqrt(\pi)) \int_x^\infty \exp(-t^2).")

(defmfun log-erfc (x)
  "gsl_sf_log_erfc_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The logarithm of the complementary error function \log(\erfc(x)).")

(defmfun erf-Z (x)
  "gsl_sf_erf_Z_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Gaussian probability density function 
  Z(x) = (1/sqrt{2\pi}) \exp(-x^2/2)}.")

(defmfun erf-Q (x)
  "gsl_sf_erf_Q_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The upper tail of the Gaussian probability function 
  Q(x) = (1/\sqrt{2\pi}) \int_x^\infty dt \exp(-t^2/2)}.")

(defmfun hazard (x)
  "gsl_sf_hazard_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The hazard function for the normal distribution.")

(save-test error-functions
  (erf 1.0d0)
  (erfc 1.0d0)
  (log-erfc 1.0d0)
  (erf-z 1.0d0)
  (erf-q 1.0d0)
  (hazard 1.0d0))

