;; Transport functions
;; Liam Healy, Mon May  1 2006 - 22:29
;; Time-stamp: <2011-10-29 23:44:51EDT transport.lisp>
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

;;;  FDL
;;; The transport functions J(n,x) are defined by the integral 
;;; representations
;;; J(n,x) := \int_0^x dt \, t^n e^t /(e^t - 1)^2.

(defmfun transport-2 (x)
  "gsl_sf_transport_2_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The transport function J(2,x).")

(defmfun transport-3 (x)
  "gsl_sf_transport_3_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The transport function J(3,x).")

(defmfun transport-4 (x)
  "gsl_sf_transport_4_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The transport function J(4,x).")

(defmfun transport-5 (x)
  "gsl_sf_transport_5_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The transport function J(5,x).")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test transport
  (transport-2 4.0d0)
  (transport-3 4.0d0)
  (transport-4 4.0d0)
  (transport-5 4.0d0))

