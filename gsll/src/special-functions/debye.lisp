;; Deybe functions
;; Liam Healy, Sun Mar 19 2006 - 14:34
;; Time-stamp: <2011-10-29 23:35:07EDT debye.lisp>
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

#|
;;; FDL
The Debye functions D_n(x) are defined by the following integral,
D_n(x) = n/x^n \int_0^x dt (t^n/(e^t - 1))
For further information see Abramowitz &
Stegun, Section 27.1.
|#

(in-package :gsl)

(defmfun debye-1 (x)
  "gsl_sf_debye_1_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The first-order Debye function D_1(x) = (1/x) \int_0^x dt (t/(e^t - 1)).")

(defmfun debye-2 (x)
  "gsl_sf_debye_2_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The second-order Debye function
   D_2(x) = (2/x^2) \int_0^x dt (t^2/(e^t - 1)).")

(defmfun debye-3 (x)
  "gsl_sf_debye_3_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The third-order Debye function
   D_3(x) = (3/x^3) \int_0^x dt (t^3/(e^t - 1)).")

(defmfun debye-4 (x)
  "gsl_sf_debye_4_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The fourth-order Debye function
  D_4(x) = (4/x^4) \int_0^x dt (t^4/(e^t - 1)).")

(save-test debye
  (debye-1 1.0d0)
  (debye-2 1.0d0)
  (debye-3 1.0d0)
  (debye-4 1.0d0))

