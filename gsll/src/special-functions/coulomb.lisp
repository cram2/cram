;; Coulumb functions
;; Liam Healy, Sat Mar 18 2006 - 23:23
;; Time-stamp: <2011-10-29 23:32:13EDT coulomb.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2010, 2011 Liam M. Healy
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

;;; /usr/include/gsl/gsl_sf_coulomb.h

;;;;****************************************************************************
;;;; Normalized Hydrogenic Bound States
;;;;****************************************************************************

(defmfun hydrogenicR-1 (x r)
  "gsl_sf_hydrogenicR_1_e"
  ((x :double) (r :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The lowest-order normalized hydrogenic bound state radial
   wavefunction R_1 := 2Z \sqrt{Z} \exp(-Z r).")

(defmfun hydrogenicR (n l x r)
  "gsl_sf_hydrogenicR_e"
  ((n :int) (l :int) (x :double) (r :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The n-th normalized hydrogenic bound state radial wavefunction,
  R_n := {2 Z^{3/2} \over n^2}  \left({2Z \over n}\right)^l
  \sqrt{(n-l-1)! \over (n+l)!} \exp(-Z r/n) L^{2l+1}_{n-l-1}(2Z/n r).
  The normalization is chosen such that the wavefunction \psi is given by 
  \psi(n,l,r) = R_n Y_{lm}.")

;;;;****************************************************************************
;;;; Coulomb Wave Functions
;;;;****************************************************************************

(defmfun coulomb-wave-FG (eta x L-F k)
  "gsl_sf_coulomb_wave_FG_e"
  ((eta :double) (x :double) (L-F :double) (k :int)
   (F (:pointer (:struct sf-result))) (Fp (:pointer (:struct sf-result)))
   (G (:pointer (:struct sf-result))) (Gp (:pointer (:struct sf-result)))
   (exp-F (:pointer :double)) (exp-G (:pointer :double)))
  :return
  ((let ((vl (multiple-value-list (values-with-errors F Fp G Gp))))
     (values-list
      (append (subseq vl 0 4)
	      (list
	       (cffi:mem-aref exp-F :double)
	       (cffi:mem-aref exp-G :double))
	      (subseq vl 4)))))
  :documentation			; FDL
  "The Coulomb wave functions F_L(\eta,x),
  G_{L-k}(\eta,x) and their derivatives F'_L(\eta,x), G'_{L-k}(\eta,x)
  with respect to x.  The parameters are restricted to L, L-k > -1/2},
  x > 0 and integer k.  Note that L itself is not restricted to being
  an integer. The results are stored in the parameters F, G for the
  function values and Fp, Gp for the derivative values.  If an
  overflow occurs, the condition 'overflow is signalled and scaling
  exponents are stored in the modifiable parameters exp-F, exp-G.")

(defmfun coulomb-wave-F-array
    (L-min eta x &optional (size-or-array *default-sf-array-size*)
	   &aux (fc-array (vdf size-or-array)))
  "gsl_sf_coulomb_wave_F_array"
  ((L-min :double) ((1- (dim0 fc-array)) :int) (eta :double) (x :double)
   ((grid:foreign-pointer fc-array) :pointer) (F-exponent (:pointer :double)))
  :outputs (fc-array)
  :return (fc-array (cffi:mem-ref F-exponent :double))
  :documentation			; FDL
  "The Coulomb wave function F_L(\eta,x) for
  L = Lmin ... Lmin + kmax, storing the results in fc-array.
  In the case of overflow the exponent is stored in the second value returned.")

(defmfun coulomb-wave-FG-array
    (L-min eta x
	   &optional (fc-size-or-array *default-sf-array-size*)
	   gc-size-or-array
	   &aux (fc-array (vdf fc-size-or-array))
	   (gc-array (vdf (or gc-size-or-array (dim0 fc-array)))))
  "gsl_sf_coulomb_wave_FG_array"
  ((L-min :double) ((1- (dim0 fc-array)) :int) (eta :double) (x :double)
   ((grid:foreign-pointer fc-array) :pointer) ((grid:foreign-pointer gc-array) :pointer)
   (F-exponent (:pointer :double)) (G-exponent (:pointer :double)))
  :outputs (fc-array gc-array)
  :return (fc-array
	   gc-array
	   (cffi:mem-ref F-exponent :double)
	   (cffi:mem-ref G-exponent :double))
  :documentation			; FDL
  "The functions F_L(\eta,x),
  G_L(\eta,x) for L = Lmin ... Lmin + kmax storing the
  results in fc_array and gc_array.  In the case of overflow the
  exponents are stored in F_exponent and G_exponent.")

(defmfun coulomb-wave-FGp-array
    (L-min eta x
	   &optional (fc-size-or-array *default-sf-array-size*)
	   fcp-size-or-array
	   gc-size-or-array
	   gcp-size-or-array
	   &aux (fc-array (vdf fc-size-or-array))
	   (fcp-array (vdf (or fcp-size-or-array (dim0 fc-array))))
	   (gc-array (vdf (or gc-size-or-array (dim0 fc-array))))
	   (gcp-array (vdf (or gcp-size-or-array (dim0 fc-array)))))
  "gsl_sf_coulomb_wave_FGp_array"
  ((L-min :double) ((1- (dim0 fc-array)) :int) (eta :double) (x :double)
   ((grid:foreign-pointer fc-array) :pointer) ((grid:foreign-pointer fcp-array) :pointer)
   ((grid:foreign-pointer gc-array) :pointer) ((grid:foreign-pointer gcp-array) :pointer)
   (F-exponent (:pointer :double)) (G-exponent (:pointer :double)))
  :outputs (fc-array fcp-array gc-array gcp-array)
  :return
  (fc-array fcp-array gc-array gcp-array
	    (cffi:mem-ref F-exponent :double) (cffi:mem-ref G-exponent :double))
  :documentation			; FDL
  "The functions F_L(\eta,x),
  G_L(\eta,x) and their derivatives F'_L(\eta,x),
  G'_L(\eta,x) for L = Lmin ... Lmin + kmax storing the
  results in fc_array, gc_array, fcp_array and gcp_array.
  In the case of overflow the exponents are stored in F_exponent
  and G_exponent.")

(defmfun coulomb-wave-sphF-array
    (L-min eta x &optional (size-or-array *default-sf-array-size*)
	   &aux (fc-array (vdf size-or-array)))
  "gsl_sf_coulomb_wave_sphF_array"
  ((L-min :double) ((1- (dim0 fc-array)) :int) (eta :double) (x :double)
   ((grid:foreign-pointer fc-array) :pointer) (F-exponent (:pointer :double)))
  :outputs (fc-array)
  :return (fc-array (cffi:mem-ref F-exponent :double))
  :documentation			; FDL
  "The Coulomb wave function divided by the argument
   F_L(\eta, x)/x for L = Lmin ... Lmin + kmax, storing the
   results in fc_array.  In the case of overflow the exponent is
   stored in F_exponent. This function reduces to spherical Bessel
   functions in the limit \eta \to 0.")

;;;;****************************************************************************
;;;; Coulomb Wave Function Normalization Constant
;;;;****************************************************************************

(defmfun coulomb-CL (L eta)
  "gsl_sf_coulomb_CL_e"
  ((L :double) (eta :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Coulomb wave function normalization constant C_L(\eta)
   for L > -1.")

(defmfun coulomb-cl-array
    (L-min eta &optional (size-or-array *default-sf-array-size*)
	   &aux (array (vdf size-or-array)))
  "gsl_sf_coulomb_CL_array"
  ((L-min :double) ((1- (dim0 array)) :int) (eta :double)
   ((grid:foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "The Coulomb wave function normalization constant C_L(\eta)
   for L = Lmin ... Lmin + kmax, Lmin > -1.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test coulomb
 (hydrogenicr-1 1.0d0 2.5d0)
 (hydrogenicr 3 1 1.0d0 2.5d0)
 (coulomb-wave-FG 0.0d0 1.0d0 2.0d0 0)
 (let ((arr (grid:make-foreign-array 'double-float :dimensions 3)))
   (coulomb-wave-F-array 0.0d0 1.0d0 2.0d0 arr)
   (grid:copy-to arr))
 (coulomb-wave-fg 1.0d0 2.0d0 2.5d0 1)
 (let ((Farr (grid:make-foreign-array 'double-float :dimensions 3))
       (Garr (grid:make-foreign-array 'double-float :dimensions 3)))
   (coulomb-wave-FG-array 1.5d0 1.0d0 1.0d0 Farr Garr)
   (append (coerce (grid:copy-to Farr) 'list) (coerce (grid:copy-to Garr) 'list)))
 (let ((arr (grid:make-foreign-array 'double-float :dimensions 3)))
   (coulomb-wave-sphF-array  0.0d0 1.0d0 2.0d0 arr) (grid:copy-to arr))
 (coulomb-cl 1.0d0 2.5d0)
 (let ((cl (grid:make-foreign-array 'double-float :dimensions 3)))
   (coulomb-CL-array 0.0d0 1.0d0 cl) (grid:copy-to cl)))
