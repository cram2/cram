;; Bessel functions
;; Liam Healy, Fri Mar 17 2006 - 18:42
;; Time-stamp: <2010-07-07 14:24:57EDT bessel.lisp>
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
;;;; Regular Cylindrical Bessel Functions
;;;;****************************************************************************

(defmfun cylindrical-bessel-J0 (x)
    "gsl_sf_bessel_J0_e"
  ((x :double) (ret sf-result))
  :documentation			; FDL
  "The regular cylindrical Bessel function of zeroth order, J_0(x).")

(defmfun cylindrical-bessel-J1 (x)
  "gsl_sf_bessel_J1_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The regular cylindrical Bessel function of first order, J_1(x).")

(export 'cylindrical-bessel-J)
(defgeneric cylindrical-bessel-J (order x)
  (:documentation			; FDL
   "The regular cylindrical Bessel function of order n, J_n(x)."))

(defmfun cylindrical-bessel-J ((n integer) x)
  "gsl_sf_bessel_Jn_e" ((n :int) (x :double) (ret sf-result))
  :definition :method)

(defmfun cylindrical-bessel-J-array-order
    (x &optional (size-or-array *default-sf-array-size*) (nmin 0)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_Jn_array"
  ((nmin :int) ((+ nmin (dim0 array) -1) :int) (x :double)
   ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "The values of the regular cylindrical Bessel functions J_n(x)
   for n from nmin to nmin+length(array)-1 inclusive.
   The values are computed using recurrence relations for efficiency,
   and therefore may differ slightly from the exact values.")

;;; Fractional order

(defmfun cylindrical-bessel-J ((nu float) x)
  "gsl_sf_bessel_Jnu_e" ((nu :double) (x :double) (ret sf-result))
  :definition :method)

(defmfun cylindrical-bessel-J-array-x (nu v &optional (mode :double))
  "gsl_sf_bessel_sequence_Jnu_e"
  ((nu :double) (mode sf-mode) ((dim0 v) :int) ((foreign-pointer v) :pointer))
  :inputs (v)
  :outputs (v)
  :documentation			; FDL
  "The regular cylindrical Bessel function of
  fractional order \nu, J_\nu(x), evaluated at a series of
  x values.  The array v contains the x values.
  They are assumed to be strictly ordered and positive.
  The array is over-written with the values of J_\nu(x_i).")

;;;;****************************************************************************
;;;; Irregular Cylindrical Bessel Functions
;;;;****************************************************************************

(defmfun cylindrical-bessel-Y0 (x)
  "gsl_sf_bessel_Y0_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The irregular cylindrical Bessel function of zeroth order, Y_0(x).")

(defmfun cylindrical-bessel-Y1 (x)
  "gsl_sf_bessel_Y1_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The irregular cylindrical Bessel function of first order, Y_1(x).")

(export 'cylindrical-bessel-Y)
(defgeneric cylindrical-bessel-Y (order x)
  (:documentation			; FDL
   "The irregular cylindrical Bessel function of order n, Y_n(x)."))

(defmfun cylindrical-bessel-Y ((n integer) x)
  "gsl_sf_bessel_Yn_e"
  ((n :int) (x :double) (ret sf-result))
  :definition :method
  :documentation			; FDL
  "The irregular cylindrical Bessel function of order n, Y_n(x).")

(defmfun cylindrical-bessel-Yn-array
    (x &optional (size-or-array *default-sf-array-size*) (nmin 0)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_Yn_array"
  ((nmin :int) ((+ nmin (dim0 array) -1) :int) (x :double)
   ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "The values of the irregular cylindrical Bessel functions
   Y_n(x) for n from from nmin to
   nmin+length(array)-1 inclusive.  The values are computed using
   recurrence relations for efficiency, and therefore may differ slightly
   from the exact values.")

;;; Irregular Bessel Function - Fractional Order

(defmfun cylindrical-bessel-Y ((nu float) x)
  "gsl_sf_bessel_Ynu_e" ((nu :double) (x :double) (ret sf-result))
  :definition :method)

;;;;****************************************************************************
;;;; Regular Modified Cylindrical Bessel Functions
;;;;****************************************************************************

(defmfun cylindrical-bessel-I0 (x)
  "gsl_sf_bessel_I0_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The regular modified cylindrical Bessel function of zeroth order, I_0(x).")

(defmfun cylindrical-bessel-I1 (x)
  "gsl_sf_bessel_I1_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The regular modified cylindrical Bessel function of first order, I_1(x).")

(export 'cylindrical-bessel-I)
(defgeneric cylindrical-bessel-I (order x)
  (:documentation			; FDL
   "The regular modified cylindrical Bessel function of order n, I_n(x)."))

(defmfun cylindrical-bessel-I ((n integer) x)
  "gsl_sf_bessel_In_e"
  ((n :int) (x :double) (ret sf-result))
  :definition :method)

(defmfun cylindrical-bessel-In-array
    (x &optional (size-or-array *default-sf-array-size*) (nmin 0)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_In_array"
  ((nmin :int) ((+ nmin (dim0 array) -1) :int) (x :double)
   ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "The values of the regular modified cylindrical Bessel functions
   I_n(x) for n from from nmin to nmin+length(array)-1 inclusive.
   The values are computed using recurrence relations for efficiency, and
   therefore may differ slightly from the exact values.")

(defmfun cylindrical-bessel-I0-scaled (x)
  "gsl_sf_bessel_I0_scaled_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled regular modified cylindrical Bessel function of zeroth order,
  \exp(-|x|) I_0(x).")

(defmfun cylindrical-bessel-I1-scaled (x)
  "gsl_sf_bessel_I1_scaled_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled regular modified cylindrical Bessel function of first order,
  \exp(-|x|) I_1(x).")

(export 'cylindrical-bessel-I-scaled)
(defgeneric cylindrical-bessel-I-scaled (order x)
  (:documentation			; FDL
   "The scaled regular modified cylindrical Bessel function of order n,
  \exp(-|x|) I_n(x)}."))

(defmfun cylindrical-bessel-I-scaled ((n integer) x)
  "gsl_sf_bessel_In_scaled_e" ((n :int) (x :double) (ret sf-result))
  :definition :method
  :documentation			; FDL
  "The scaled regular modified cylindrical Bessel function of order n,
  \exp(-|x|) I_n(x)}.")

(defmfun cylindrical-bessel-In-scaled-array
    (x &optional (size-or-array *default-sf-array-size*) (nmin 0)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_In_scaled_array"
  ((nmin :int) ((+ nmin (dim0 array) -1) :int) (x :double)
   ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  ;; Bug in original documentation?  doesn't say "modified"
  "The values of the scaled regular modified cylindrical Bessel
  functions I_n(x) for n from from nmin to nmin+length(array)-1
  inclusive. The values are computed using recurrence
  relations for efficiency, and therefore may differ slightly from the
  exact values.")

;;; Regular Modified Bessel Functions - Fractional Order

(defmfun cylindrical-bessel-I ((nu float) x)
  "gsl_sf_bessel_Inu_e" ((nu :double) (x :double) (ret sf-result))
  :definition :method
  :documentation			; FDL
  "The regular modified Bessel function of fractional order
  \nu, I_\nu(x) for x>0, \nu>0.")

(defmfun cylindrical-bessel-I-scaled ((nu float) x)
  "gsl_sf_bessel_Inu_scaled_e" ((nu :double) (x :double) (ret sf-result))
  :definition :method
  :documentation			; FDL
  "The scaled regular modified Bessel function of fractional order
  \nu, \exp(-|x|)I_\nu(x) for x>0, \nu>0.")

;;;;****************************************************************************
;;;; Irregular Modified Cylindrical Bessel Functions
;;;;****************************************************************************

(defmfun cylindrical-bessel-K0 (x)
  "gsl_sf_bessel_K0_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The irregular modified cylindrical Bessel function of zeroth order,
  K_0(x).")

(defmfun cylindrical-bessel-K1 (x)
  "gsl_sf_bessel_K1_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The irregular modified cylindrical Bessel function of first order, K_1(x).")

(export 'cylindrical-bessel-K)
(defgeneric cylindrical-bessel-K (order x)
  (:documentation			; FDL
   "The irregular modified cylindrical Bessel function of order n, K_n(x)."))

(defmfun cylindrical-bessel-K ((n integer) x)
  "gsl_sf_bessel_Kn_e" ((n :int) (x :double) (ret sf-result))
  :definition :method)

(defmfun cylindrical-bessel-Kn-array
    (x &optional (size-or-array *default-sf-array-size*) (nmin 0)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_Kn_array"
  ((nmin :int) ((+ nmin (dim0 array) -1) :int) (x :double)
   ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "The values of the irregular modified cylindrical Bessel functions
   K_n(x) for n from from nmin to nmin+length(array)-1 inclusive.
   The values are computed using recurrence relations for efficiency, and
   therefore may differ slightly from the exact values.")

(defmfun cylindrical-bessel-K0-scaled (x)
  "gsl_sf_bessel_K0_scaled_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled irregular modified cylindrical Bessel function of zeroth order,
  \exp(-|x|) K_0(x).")

(defmfun cylindrical-bessel-K1-scaled (x)
  "gsl_sf_bessel_K1_scaled_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled irregular modified cylindrical Bessel function of first order,
   \exp(-|x|) K_1(x).")

(export 'cylindrical-bessel-K-scaled)
(defgeneric cylindrical-bessel-K-scaled (order x)
  (:documentation			; FDL
   "The scaled irregular modified cylindrical Bessel function of order n,
  \exp(-|x|) K_n(x)."))

(defmfun cylindrical-bessel-K-scaled ((n integer) x)
  "gsl_sf_bessel_Kn_scaled_e" ((n :int) (x :double) (ret sf-result))
  :definition :method)

(defmfun cylindrical-bessel-Kn-scaled-array
    (x &optional (size-or-array *default-sf-array-size*) (nmin 0)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_Kn_scaled_array"
  ((nmin :int) ((+ nmin (dim0 array) -1) :int) (x :double)
   ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "The values of the scaled irregular cylindrical
   Bessel functions \exp(x) K_n(x) for n from from nmin to
   nmin+length(array)-1 inclusive.
   The start of the range nmin must be positive
   or zero.  The domain of the function is x>0. The values are
   computed using recurrence relations for efficiency, and therefore
   may differ slightly from the exact values.")

;;; Irregular Modified Bessel Functions - Fractional Order

(defmfun cylindrical-bessel-K ((nu float) x)
  "gsl_sf_bessel_Knu_e" ((nu :double) (x :double) (ret sf-result))
  :definition :method
  :documentation			; FDL
  "The irregular modified Bessel function of fractional order \nu,
   K_\nu(x) for x>0, \nu>0.")

(defmfun bessel-lnKnu (nu x)
  "gsl_sf_bessel_lnKnu_e" ((nu :double) (x :double) (ret sf-result))
  :documentation			; FDL
  "The logarithm of the irregular modified Bessel function of fractional
   order \nu, \ln(K_\nu(x)) for x>0, \nu>0.")

(defmfun cylindrical-bessel-K-scaled ((nu float) x)
  "gsl_sf_bessel_Knu_scaled_e" ((nu :double) (x :double) (ret sf-result))
  :definition :method
  :documentation			; FDL
  "The scaled irregular modified Bessel function of fractional order
   \nu, \exp(+|x|) K_\nu(x) for x>0, \nu>0.")

;;;;****************************************************************************
;;;; Regular Spherical Bessel Functions
;;;;****************************************************************************

(defmfun spherical-bessel-j0 (x)
  "gsl_sf_bessel_j0_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The regular spherical Bessel function of zeroth order, j_0(x) = \sin(x)/x.")

(defmfun spherical-bessel-j1 (x)
  "gsl_sf_bessel_j1_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The regular spherical Bessel function of first order, j_1(x)
   = (\sin(x)/x - \cos(x))/x.")

(defmfun spherical-bessel-j2 (x)
  "gsl_sf_bessel_j2_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The regular spherical Bessel function of second order, j_2(x)
   = ((3/x^2 - 1)\sin(x) - 3\cos(x)/x)/x.")

(defmfun spherical-bessel-jl (l x)
  "gsl_sf_bessel_jl_e" ((l :int) (x :double) (ret sf-result))
  :documentation			; FDL
  "The regular spherical Bessel function of order
   l, j_l(x), for l >= 0 and x >= 0.")

(defmfun spherical-bessel-jl-array
    (x &optional (size-or-array *default-sf-array-size*)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_jl_array"
  (((1- (dim0 array)) :int) (x :double) ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "The values of the regular spherical Bessel
  functions j_l(x) for l from 0 to length(array)-1 and x >= 0.
  The values are computed using recurrence relations for
  efficiency, and therefore may differ slightly from the exact values.")

(defmfun spherical-bessel-jl-steed-array
    (x &optional (size-or-array *default-sf-array-size*)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_jl_steed_array"
  (((1- (dim0 array)) :int) (x :double) ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "Uses Steed's method to compute the values of the regular spherical
  Bessel functions j_l(x) for l from 0 to length(array)-1
  inclusive for x >= 0. The Steed/Barnett algorithm is described in
  Comp. Phys. Comm. 21, 297 (1981).  Steed's method is more
  stable than the recurrence used in the other functions but is also
  slower.")

;;;;****************************************************************************
;;;; Irregular Spherical Bessel Functions
;;;;****************************************************************************

(defmfun spherical-bessel-y0 (x)
  "gsl_sf_bessel_y0_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The irregular spherical Bessel function of zeroth order,
  y_0(x) = -\cos(x)/x.")

(defmfun spherical-bessel-y1 (x)
  "gsl_sf_bessel_y1_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The irregular spherical Bessel function of first order,
  y_1(x) = -(\cos(x)/x + \sin(x))/x.")

(defmfun spherical-bessel-y2 (x)
  "gsl_sf_bessel_y2_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The irregular spherical Bessel function of second order,
  y_2(x) = (-3/x^3 + 1/x)\cos(x) - (3/x^2)\sin(x).")

(defmfun spherical-bessel-yl (l x)
  "gsl_sf_bessel_yl_e" ((l :int) (x :double) (ret sf-result))
  :documentation			; FDL
  "The irregular spherical Bessel function of order l, y_l(x), for l >= 0.")

(defmfun spherical-bessel-yl-array
    (x &optional (size-or-array *default-sf-array-size*)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_yl_array"
  (((1- (dim0 array)) :int) (x :double) ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "The irregular spherical Bessel functions y_l(x) for l
  from 0 to length(array)-1.  The values are computed using
  recurrence relations for efficiency,
  and therefore may differ slightly from the exact values.")

;;;;****************************************************************************
;;;; Regular Modified Spherical Bessel Functions
;;;;****************************************************************************

(defmfun spherical-bessel-i0-scaled (x)
  "gsl_sf_bessel_i0_scaled_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled regular modified spherical Bessel function of zeroth
  order, \exp(-|x|) i_0(x).")

(defmfun spherical-bessel-i1-scaled (x)
  "gsl_sf_bessel_i1_scaled_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled regular modified spherical Bessel function of first order,
  \exp(-|x|) i_1(x).")

(defmfun spherical-bessel-i2-scaled (x)
  "gsl_sf_bessel_i2_scaled_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled regular modified spherical Bessel function of second order,
   \exp(-|x|) i_2(x).")

(defmfun spherical-bessel-il-scaled (n x)
  "gsl_sf_bessel_il_scaled_e" ((n :int) (x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled regular modified spherical Bessel function of order l,
   \exp(-|x|) i_l(x).")

(defmfun spherical-bessel-il-scaled-array
    (x &optional (size-or-array *default-sf-array-size*)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_il_scaled_array"
  (((1- (dim0 array)) :int) (x :double) ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "The values of the scaled regular modified cylindrical Bessel
  functions \exp(-|x|) i_l(x) for l from 0 to length(array)-1
  inclusive.  The values are computed using recurrence relations
  for efficiency, and therefore may differ slightly from the exact values.")

;;;;****************************************************************************
;;;; Irregular Modified Spherical Bessel Functions
;;;;****************************************************************************

(defmfun spherical-bessel-k0-scaled (x)
  "gsl_sf_bessel_k0_scaled_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled irregular modified spherical Bessel function of zeroth
  order, \exp(x) k_0(x), for x>0.")

(defmfun spherical-bessel-k1-scaled (x)
  "gsl_sf_bessel_k1_scaled_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled irregular modified spherical Bessel function of first order,
   \exp(x) k_1(x), for x>0.")

(defmfun spherical-bessel-k2-scaled (x)
  "gsl_sf_bessel_k2_scaled_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled irregular modified spherical Bessel function of second order,
  \exp(x) k_2(x), for x>0.")

(defmfun spherical-bessel-kl-scaled (n x)
  "gsl_sf_bessel_il_scaled_e" ((n :int) (x :double) (ret sf-result))
  :documentation			; FDL
  "The scaled irregular modified spherical Bessel function of order l,
   \exp(x) k_l(x), for x>0.")

(defmfun spherical-bessel-kl-scaled-array
    (x &optional (size-or-array *default-sf-array-size*)
       &aux (array (vdf size-or-array)))
  "gsl_sf_bessel_kl_scaled_array"
  (((1- (dim0 array)) :int) (x :double) ((foreign-pointer array) :pointer))
  :outputs (array)
  :documentation			; FDL
  "The values of the scaled irregular modified spherical Bessel
  functions \exp(x) k_l(x) for l from 0 to length(array)-1
  inclusive x>0.  The values are computed using recurrence
  relations for efficiency, and therefore may differ slightly
  from the exact values.")

;;;;****************************************************************************
;;;; Zeros of Regular Bessel Functions
;;;;****************************************************************************

(defmfun bessel-zero-J0 (s)
  "gsl_sf_bessel_zero_J0_e" ((s :int) (ret sf-result))
  :documentation			; FDL
  "The location of the s-th positive zero of the Bessel function
  J_0(x).")

(defmfun bessel-zero-J1 (s)
  "gsl_sf_bessel_zero_J1_e" ((s :int) (ret sf-result))
  :documentation			; FDL
  "The location of the s-th positive zero of the Bessel function
  J_1(x).")

(defmfun bessel-zero-Jnu (nu s)
  "gsl_sf_bessel_zero_Jnu_e" ((nu :double) (s :int) (ret sf-result))
  :documentation			; FDL
  "These routines compute the location of the s-th positive zero
  of the Bessel function J_\nu(x).  The current implementation
  does not support negative values of nu.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test bessel
	   (cylindrical-bessel-J0 4.0d0)
	   (cylindrical-bessel-J1 4.0d0)
	   (cylindrical-bessel-J 2 4.0d0)
	   (grid:copy-to (cylindrical-bessel-J-array-order 2.0d0 4 2))
	   (cylindrical-bessel-Y0 4.0d0)
	   (cylindrical-bessel-Y1 4.0d0)
	   (cylindrical-bessel-Y 3 4.0d0)
	   (let ((besarr (grid:make-foreign-array 'double-float :dimensions 4)))
	     (cylindrical-bessel-Yn-array 2.0d0 besarr 2)
	     (grid:copy-to besarr))
	   (cylindrical-bessel-I0 4.0d0)
	   (cylindrical-bessel-I1 4.0d0)
	   (cylindrical-bessel-I 3 4.0d0)
	   (cylindrical-bessel-I 3.0d0 4.0d0)
	   (let ((besarr (grid:make-foreign-array 'double-float :dimensions 4)))
	     (cylindrical-bessel-In-array 2.0d0 besarr 2)
	     (grid:copy-to besarr))
	   (cylindrical-bessel-I0-scaled 4.0d0)
	   (cylindrical-bessel-I1-scaled 4.0d0)
	   (cylindrical-bessel-I-scaled 3 4.0d0)
	   (cylindrical-bessel-I-scaled 3.0d0 4.0d0)
	   (let ((besarr (grid:make-foreign-array 'double-float :dimensions 4)))
	     (cylindrical-bessel-In-scaled-array 2.0d0 besarr 2)
	     (grid:copy-to besarr))
	   (cylindrical-bessel-K0 4.0d0)
	   (cylindrical-bessel-K1 4.0d0)
	   (cylindrical-bessel-K 2 4.0d0)
	   (cylindrical-bessel-K 3.0d0 4.0d0)
	   (let ((besarr (grid:make-foreign-array 'double-float :dimensions 4)))
	     (cylindrical-bessel-Kn-array 2.0d0 besarr 2)
	     (grid:copy-to besarr))
	   (cylindrical-bessel-K0-scaled 4.0d0)
	   (cylindrical-bessel-K1-scaled 4.0d0)
	   (cylindrical-bessel-K-scaled 2 4.0d0)
	   (cylindrical-bessel-K-scaled 3.0d0 4.0d0)
	   (let ((besarr (grid:make-foreign-array 'double-float :dimensions 4)))
	     (cylindrical-bessel-Kn-array 2.0d0 besarr 2)
	     (grid:copy-to besarr))
	   (spherical-bessel-j0 4.0d0)
	   (spherical-bessel-j1 4.0d0)
	   (spherical-bessel-j2 4.0d0)
	   (spherical-bessel-jl 3 4.0d0)
	   (let ((besarr (grid:make-foreign-array 'double-float :dimensions 4)))
	     (spherical-bessel-jl-array 4.0d0 besarr)
	     (grid:copy-to besarr))
	   (let ((besarr (grid:make-foreign-array 'double-float :dimensions 4)))
	     (spherical-bessel-jl-steed-array 4.0d0 besarr)
	     (grid:copy-to besarr))
	   (spherical-bessel-y0 4.0d0)
	   (spherical-bessel-y1 4.0d0)
	   (spherical-bessel-y2 4.0d0)
	   (spherical-bessel-yl 2 4.0d0)
	   (let ((besarr (grid:make-foreign-array 'double-float :dimensions 4)))
	     (spherical-bessel-yl-array 4.0d0 besarr)
	     (grid:copy-to besarr))
	   (spherical-bessel-i0-scaled 4.0d0)
	   (spherical-bessel-i1-scaled 4.0d0)
	   (spherical-bessel-i2-scaled 4.0d0)
	   (spherical-bessel-il-scaled 3 4.0d0)
	   (let ((besarr (grid:make-foreign-array 'double-float :dimensions 4)))
	     (spherical-bessel-il-scaled-array 4.0d0 besarr)
	     (grid:copy-to besarr))
	   (spherical-bessel-k0-scaled 4.0d0)
	   (spherical-bessel-k1-scaled 4.0d0)

	   (spherical-bessel-k2-scaled 4.0d0)
	   (spherical-bessel-kl-scaled 3 4.0d0)
	   (let ((besarr (grid:make-foreign-array 'double-float :dimensions 4)))
	     (spherical-bessel-kl-scaled-array 4.0d0 besarr)
	     (grid:copy-to besarr))
	   (cylindrical-bessel-J 3.0d0 4.0d0)
	   (grid:copy-to (cylindrical-bessel-J-array-x 0.5d0 #m(1.0d0 2.0d0 3.0d0)))
	   (cylindrical-bessel-Y 3.0d0 4.0d0)
	   (bessel-lnKnu 3.0d0 4.0d0)
	   (bessel-zero-J0 5)
	   (bessel-zero-J1 5)
	   (bessel-zero-Jnu 2.0d0 5))
