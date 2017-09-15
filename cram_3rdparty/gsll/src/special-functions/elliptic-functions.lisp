;; Jacobian elliptic functions
;; Liam Healy, Mon Mar 20 2006 - 22:21
;; Time-stamp: <2010-08-07 21:41:12EDT elliptic-functions.lisp>
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

;; /usr/include/gsl/gsl_sf_elljac.h

(defmfun jacobian-elliptic-functions (u m)
  "gsl_sf_elljac_e"
  ((u :double) (m :double)
   (sn (:pointer :double)) (cn (:pointer :double)) (dn (:pointer :double)))
  :documentation			; FDL
  "The Jacobian elliptic functions sn(u|m),
  cn(u|m), dn(u|m) computed by descending Landen transformations.")

(defvar *elljac-K* (/ dpi (* 2.0d0 0.9741726903999478375938128316d0)))
(defvar *elljac-A* (/ (sqrt (1+ (sqrt 0.9d0)))))
(defvar *elljac-B* (/ (expt 0.9d0 1/4) (sqrt (1+ (sqrt 0.9d0)))))
(defvar *elljac-C* (expt 0.9d0 1/4))
(defvar *elljac-C2* (sqrt 0.9d0))

(save-test
 elliptic-functions
 ;; Tests from gsl/specfunc/test_sf.c
 (jacobian-elliptic-functions 0.5d0 0.5d0)
 (jacobian-elliptic-functions 1.0d0 0.3d0)
 (jacobian-elliptic-functions 1.0d0 0.6d0)
 (jacobian-elliptic-functions 2.0d0 0.999999d0)
 (jacobian-elliptic-functions 1.69695970624443d0 0.270378013104138d0)
 (jacobian-elliptic-functions 0.0d0 0.1d0)
 (jacobian-elliptic-functions -1.0d-10 0.1d0)
 (jacobian-elliptic-functions 1.0d-10 0.1d0)
 (jacobian-elliptic-functions 1.0d-30 0.1d0)
 (jacobian-elliptic-functions (- (/ *elljac-K* 2.0d0) 1.0d-10) 0.1d0)
 (jacobian-elliptic-functions (/ *elljac-K* 2.0d0) 0.1d0)
 (jacobian-elliptic-functions (+ (/ *elljac-K* 2.0d0) 1.0d-10) 0.1d0)
 (jacobian-elliptic-functions (- *elljac-K* 1.0d-10) 0.1d0)
 (jacobian-elliptic-functions *elljac-K* 0.1d0)
 (jacobian-elliptic-functions (+ *elljac-K* 1.0d-10) 0.1d0)
 (jacobian-elliptic-functions (* 3/2 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (- (* 2 *elljac-K*) 1.0d-10) 0.1d0)
 (jacobian-elliptic-functions (* 2 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (+ (* 2 *elljac-K*) 1.0d-10) 0.1d0)
 (jacobian-elliptic-functions (* 5/2 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (- (* 3 *elljac-K*) 1.0d-10) 0.1d0)
 (jacobian-elliptic-functions (* 3 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (+ (* 3 *elljac-K*) 1.0d-10) 0.1d0)
 (jacobian-elliptic-functions (* 7/2 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (- (* 4 *elljac-K*) 1.0d-10) 0.1d0)
 (jacobian-elliptic-functions (* 4 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (* 9/2 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (* -1/2 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (* -1 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (* -3/2 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (* -2 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (* -5/2 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (* -3 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (* -7/2 *elljac-K*) 0.1d0)
 (jacobian-elliptic-functions (* -4 *elljac-K*) 0.1d0)
 ;; Signal an error
 (jacobian-elliptic-functions 0.61802d0 1.5d0))
