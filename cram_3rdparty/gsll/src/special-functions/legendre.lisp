;; Legendre functions
;; Liam Healy, Sat Apr 29 2006 - 19:16
;; Time-stamp: <2016-08-07 19:06:40EDT legendre.lisp>
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

;;; legendre-Plm-deriv-array same answer as legendre-Plm-array?

;;;;****************************************************************************
;;;; Legendre polynomials
;;;;****************************************************************************

(defmfun legendre-P1 (x)
  "gsl_sf_legendre_P1_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Legendre polynomials P_1(x) using an explicit
   representation.")

(defmfun legendre-P2 (x)
  "gsl_sf_legendre_P2_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Legendre polynomials P_2(x) using an explicit
   representation.")

(defmfun legendre-P3 (x)
  "gsl_sf_legendre_P3_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Legendre polynomials P_3(x) using an explicit
   representation.")

(defmfun legendre-Pl (l x)
  "gsl_sf_legendre_Pl_e"
  ((l :int) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Legendre polynomial P_l(x) for a specific value of l,
   x subject to l >= 0, |x| <= 1.")

(defmfun legendre-Pl-array
    (x &optional (size-or-array *default-sf-array-size*)
       &aux (array (vdf size-or-array)))
  "gsl_sf_legendre_Pl_array"
  (((1- (dim0 array)) :int) (x :double) ((grid:foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "Compute an array of Legendre polynomials
  P_l(x) for l = 0, ..., length(array), |x| <= 1.")

(defmfun legendre-Pl-deriv-array
    (x &optional (size-or-array *default-sf-array-size*)
       &aux (array (vdf size-or-array)))
  "gsl_sf_legendre_Pl_deriv_array"
  (((1- (dim0 array)) :int) (x :double) ((grid:foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "Compute an array of Legendre polynomials derivatives
  dP_l(x)/dx, for l = 0, ...,  length(array), |x| <= 1.")

(defmfun legendre-Q0 (x)
  "gsl_sf_legendre_Q0_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Legendre function Q_0(x) for x > -1,
   x /= 1.")

(defmfun legendre-Q1 (x)
  "gsl_sf_legendre_Q1_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Legendre function Q_1(x) for x > -1,
   x /= 1.")

(defmfun legendre-Ql (l x)
  "gsl_sf_legendre_Ql_e" ((l :int) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Legendre function Q_l(x) for x > -1, x /= 1, l >= 0.")

;;;;****************************************************************************
;;;; Associated Legendre Polynomials and Spherical Harmonics
;;;;****************************************************************************

;;; FDL
;;; The following functions compute the associated Legendre Polynomials
;;; P_l^m(x).  Note that this function grows combinatorially with
;;; l and can overflow for l larger than about 150.  There is
;;; no trouble for small m, but overflow occurs when m and
;;; l are both large.  Rather than allow overflows, these functions
;;; refuse to calculate P_l^m(x) and signal 'overflow when
;;; they can sense that l and m are too big.

;;; If you want to calculate a spherical harmonic, then do not use
;;; these functions.  Instead use legendre-sphPlm below,
;;; which uses a similar recursion, but with the normalized functions.

(defmfun legendre-Plm (l m x)
  "gsl_sf_legendre_Plm_e"
  ((l :int) (m :int) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; GSL texi
  "The associated Legendre polynomial
   P_l^m(x) for m >= 0, l >= m, |x| <= 1.")

#-gsl2
(defmfun legendre-Plm-array
    (m x &optional (size-or-array *default-sf-array-size*)
       &aux (array (vdf size-or-array)))
  "gsl_sf_legendre_Plm_array"
  (((+ (dim0 array) m -1) :int) (m :int) (x :double)
   ((grid:foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; GSL texi
  "An array of Legendre polynomials
    P_l^m(x), for m >= 0, 
    l = |m|, ..., |m|+length(array)-1} and |x| <= 1.")

#-gsl2
(defmfun legendre-Plm-deriv-array
    (m x &optional (values-size-or-array *default-sf-array-size*)
       (derivatives-size-or-array *default-sf-array-size*)
       &aux (values (vdf values-size-or-array))
       (derivatives (vdf derivatives-size-or-array)))
  "gsl_sf_legendre_Plm_deriv_array"
  (((+ (dim0 values) m -1) :int) (m :int) (x :double)
   ((grid:foreign-pointer values) :pointer) ((grid:foreign-pointer derivatives) :pointer))
  :outputs (values derivatives)
  :documentation			; GSL texi
  "An array of Legendre polynomials
    values and derivatives dP_l^m(x)/dx for m >= 0, 
    l = |m|, ..., length(values) and |x| <= 1.")

(defmfun legendre-sphPlm (l m x)
  "gsl_sf_legendre_sphPlm_e"
  ((l :int) (m :int) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; GSL texi
  "The normalized associated Legendre polynomial
   \sqrt{(2l+1)/(4\pi) \sqrt{(l-m)!/(l+m)!} P_l^m(x) suitable
   for use in spherical harmonics.  The parameters must satisfy
   m >= 0, l >= m, |x| <= 1.  These routines avoid the overflows
   that occur for the standard normalization of P_l^m(x).")

#-gsl2
(defmfun legendre-sphPlm-array
    (m x &optional (size-or-array *default-sf-array-size*)
       &aux (array (vdf size-or-array)))
  "gsl_sf_legendre_sphPlm_array"
  (((+ (dim0 array) m -1) :int) (m :int) (x :double)
   ((grid:foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; GSL texi
  "An array of normalized associated Legendre functions
   \sqrt(2l+1)/(4\pi) \sqrt(l-m)!/(l+m)! P_l^m(x),
   for m >= 0, l = |m|, ..., length(array)}, |x| <= 1.0.")

#-gsl2
(defmfun legendre-sphPlm-deriv-array
    (m x &optional (values-size-or-array *default-sf-array-size*)
       (derivatives-size-or-array *default-sf-array-size*)
       &aux (values (vdf values-size-or-array))
       (derivatives (vdf derivatives-size-or-array)))
  "gsl_sf_legendre_sphPlm_deriv_array"
  (((+ (dim0 values) m -1) :int) (m :int) (x :double)
   ((grid:foreign-pointer values) :pointer)
   ((grid:foreign-pointer derivatives) :pointer))
  :outputs (values derivatives)
  :documentation			; GSL texi
  "An array of normalized associated Legendre functions
   values and derivatives for m >= 0,
   l = |m|, ..., length(array)}, |x| <= 1.0.")

#-gsl2
(defmfun legendre-array-size (lmax m)
  "gsl_sf_legendre_array_size" ((lmax :int) (m :int))
  :documentation			; GSL texi
  "The size of result array needed for the array
   versions of P_l^m(x), lmax - m + 1."
  :c-return :int)

;;;;****************************************************************************
;;;; Conical Functions
;;;;****************************************************************************

;;; FDL
;;; The Conical Functions P^\mu_{-(1/2)+i\lambda}(x)} and 
;;; Q^\mu_{-(1/2)+i\lambda}
;;; are described in Abramowitz & Stegun, Section 8.12.

(defmfun legendre-conicalP-half (lambda x)
  "gsl_sf_conicalP_half_e"
  ((lambda :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The irregular Spherical Conical Function
   P^{1/2}_{-1/2 + i \lambda}(x) for x > -1.")

(defmfun legendre-conicalP-mhalf (lambda x)
  "gsl_sf_conicalP_mhalf_e"
  ((lambda :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The regular Spherical Conical Function
   P^{-1/2}_{-1/2 + i \lambda}(x) for x > -1.")

(defmfun legendre-conicalP-0 (lambda x)
  "gsl_sf_conicalP_0_e"
  ((lambda :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The conical function P^0_{-1/2 + i \lambda(x)} for x > -1.")

(defmfun legendre-conicalP-1 (lambda x)
  "gsl_sf_conicalP_1_e"
  ((lambda :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The conical function 
   P^1_{-1/2 + i \lambda}(x)} for x > -1.")

(defmfun legendre-regular-spherical-conical (l lambda x)
  "gsl_sf_conicalP_sph_reg_e"
  ((l :int) (lambda :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Regular Spherical Conical Function
   P^{-1/2-l}_{-1/2 + i \lambda}(x) for x > -1, l >= -1.")

(defmfun legendre-regular-cylindrical-conical (l lambda x)
  "gsl_sf_conicalP_cyl_reg_e"
  ((l :int) (lambda :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Regular Cylindrical Conical Function
   P^{-m}_{-1/2 + i \lambda}(x) for x > -1, m >= -1.")

;;;;****************************************************************************
;;;; Radial Functions for Hyperbolic Space
;;;;****************************************************************************

;;; FDL
;;; The following spherical functions are specializations of Legendre
;;; functions which give the regular eigenfunctions of the Laplacian
;;; on a 3-dimensional hyperbolic space H3d.  Of particular interest
;;; is the flat limit, \lambda \to \infty, \eta \to 0, \lambda\eta
;;; fixed.

(defmfun legendre-H3d-0 (lambda eta)
  "gsl_sf_legendre_H3d_0_e"
  ((lambda :double) (eta :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The zeroth radial eigenfunction of the Laplacian on the
   3-dimensional hyperbolic space,
   L^{H3d}_0(\lambda,\eta) := \sin(\lambda\eta)/(\lambda\sinh(\eta))
   for \eta >= 0. In the flat limit this takes the form
   L^{H3d}_0(\lambda,\eta) = j_0(\lambda\eta).")

(defmfun legendre-H3d-1 (lambda eta)
  "gsl_sf_legendre_H3d_1_e"
  ((lambda :double) (eta :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The first radial eigenfunction of the Laplacian on
   the 3-dimensional hyperbolic space,
   L^{H3d}_1(\lambda,\eta) := 1/\sqrt{\lambda^2 + 1}
   \sin(\lambda \eta)/(\lambda \sinh(\eta)) (\coth(\eta) - \lambda \cot(\lambda\eta))}
   for \eta >= 0.  In the flat limit this takes the form 
   L^{H3d}_1(\lambda,\eta) = j_1(\lambda\eta)}.")

(defmfun legendre-H3d (l lambda eta)
  "gsl_sf_legendre_H3d_e"
  ((l :int) (lambda :double) (eta :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The l-th radial eigenfunction of the
   Laplacian on the 3-dimensional hyperbolic space
   \eta >= 0, l >= 0.  In the flat limit this takes the form
   L^{H3d}_l(\lambda,\eta) = j_l(\lambda\eta).")

(defmfun legendre-H3d-array
    (lambda eta &optional (size-or-array *default-sf-array-size*)
	    &aux (array (vdf size-or-array)))
  "gsl_sf_legendre_H3d_array"
  (((1- (dim0 array)) :int) (lambda :double) (eta :double)
   ((grid:foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "An array of radial eigenfunctions
   L^{H3d}_l(\lambda, \eta) for 0 <= l <= length(array).")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test legendre
  (legendre-P1 0.3d0)
  (legendre-P2 0.3d0)
  (legendre-P3 0.3d0)
  (legendre-Pl -4 0.3d0)
  (legendre-Pl 4 3.0d0)
  (legendre-Pl 4 0.3d0)
  (let ((arr (grid:make-foreign-array 'double-float :dimensions 4)))
      (legendre-Pl-array 0.5d0 arr)
      (grid:copy-to arr))
  (legendre-Q0 3.3d0)
  (legendre-Q1 3.3d0)
  (legendre-Ql 2 3.3d0)
  (legendre-Plm 4 3 0.5d0)
  (let ((arr (grid:make-foreign-array 'double-float :dimensions 4)))
      (legendre-Plm-array 2 0.5d0 arr)
      (grid:copy-to arr))
  (let ((val (grid:make-foreign-array 'double-float :dimensions 4))
	(deriv (grid:make-foreign-array 'double-float :dimensions 4)))
      (legendre-Plm-deriv-array 2 0.5d0 val deriv)
      (grid:copy-to deriv))
  (legendre-sphplm 1200 1100 0.3d0)
  (let ((arr (grid:make-foreign-array 'double-float :dimensions 4)))
      (legendre-sphPlm-array 4 0.5d0 arr)
      (grid:copy-to arr))
  (let ((val (grid:make-foreign-array 'double-float :dimensions 4))
	   (deriv (grid:make-foreign-array 'double-float :dimensions 4)))
	(legendre-sphPlm-deriv-array 4 0.5d0 val deriv)
	(grid:copy-to deriv))
  (legendre-conicalp-half 3.5d0 10.0d0)
  (legendre-conicalp-mhalf 3.5d0 10.0d0)
  (legendre-conicalp-0 3.5d0 10.0d0)
  (legendre-conicalp-1 3.5d0 10.0d0)
  (legendre-regular-spherical-conical 3 3.5d0 10.0d0)
  (legendre-regular-cylindrical-conical 3 3.5d0 10.0d0)
  (legendre-h3d-0 1.0d0 0.5d0)
  (legendre-h3d-1 1.0d0 0.5d0)
  (legendre-h3d 4 1.0d0 0.5d0)
  (let ((arr (grid:make-foreign-array 'double-float :dimensions 4)))
      (legendre-h3d-array 1.0d0 0.5d0 arr)
      (grid:copy-to arr)))
