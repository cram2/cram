;; Lambert's W functions
;; Liam Healy, Fri Apr 28 2006 - 20:40
;; Time-stamp: <2011-10-29 23:41:05EDT lambert.lisp>
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

;;; FDL
;;; Lambert's W functions, W(x), are defined to be solutions
;;; of the equation W(x) exp(W(x)) = x. This function has
;;; multiple branches for x < 0; however, it has only
;;; two real-valued branches. We define W_0(x) to be the
;;; principal branch, where W > -1 for x < 0, and 
;;; W_{-1}(x)
;;; W_{-1}(x) to be the other real branch, where
;;; W < -1 for x < 0.  

(defmfun lambert-W0 (x)
  "gsl_sf_lambert_W0_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The principal branch of the Lambert W function, W_0(x).")

(defmfun lambert-Wm1 (x)
  "gsl_sf_lambert_Wm1_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The secondary real-valued branch of the Lambert W function, 
   W_{-1}(x).")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test lambert
  (lambert-W0 1.0d0)
  (lambert-Wm1 1.0d0))

