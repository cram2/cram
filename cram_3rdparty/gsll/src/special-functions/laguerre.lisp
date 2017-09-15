;; Laguerre polynomials
;; Liam Healy, Fri Apr 28 2006 - 20:40
;; Time-stamp: <2011-10-29 23:40:48EDT laguerre.lisp>
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

(defmfun laguerre-1 (a x)
  "gsl_sf_laguerre_1_e"
  ((a :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The generalized Laguerre polynomial L^a_1(x) using
   explicit representations.")

(defmfun laguerre-2 (a x)
  "gsl_sf_laguerre_2_e"
  ((a :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The generalized Laguerre polynomial
   L^a_2(x) using explicit representations.")

(defmfun laguerre-3 (a x)
  "gsl_sf_laguerre_3_e"
  ((a :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The generalized Laguerre polynomial
   L^a_3(x) using explicit representations.")

(defmfun laguerre (n a x)
  "gsl_sf_laguerre_n_e"
  ((n :int) (a :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The generalized Laguerre polynomials L^a_n(x) for a > -1, n >= 0.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

;;; Don't slime-macroexpand-1, the last one will produce an error that
;;; shouldn't be there.

(save-test laguerre
  (laguerre-1 1.0d0 3.0d0)
  (laguerre-2 1.0d0 3.0d0)
  (laguerre-3 1.0d0 3.0d0)
  (laguerre 4 1.0d0 3.0d0))

