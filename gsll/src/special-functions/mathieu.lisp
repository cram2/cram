;; Mathieu functions
;; Liam Healy 2009-02-16 16:30:59EST mathieu.lisp
;; Time-stamp: <2010-07-15 10:35:18EDT mathieu.lisp>
;;
;; Copyright 2009, 2010 Liam M. Healy
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
;;;; Mathieu workspace
;;;;****************************************************************************

(defmobject mathieu "gsl_sf_mathieu"
  ((n sizet) (qmax :double))
  "workspace for Mathieu functions"
  :gsl-version (1 9)
  :documentation "Make a workspace needed for some Mathieu functions.")

;;;;****************************************************************************
;;;; Characteristic values
;;;;****************************************************************************

(defmfun mathieu-a (n q)
  "gsl_sf_mathieu_a"
  ((n :int) (q :double) (ret sf-result))
  :gsl-version (1 9)
  :documentation			; FDL
  "Compute the characteristic value a_n(q) of the Mathieu function
  ce_n(q,x) respectively.")

(defmfun mathieu-b (n q)
  "gsl_sf_mathieu_b"
  ((n :int) (q :double) (ret sf-result))
  :gsl-version (1 9)
  :documentation			; FDL
  "Compute the characteristic values b_n(q) of the Mathieu
  function se_n(q,x), respectively.")

(defmfun mathieu-a-array
    (q &optional
       size-or-array
       (minimum-order 0)
       (workspace
	(make-mathieu (+ minimum-order (vdf-size size-or-array)) q))
       &aux (size (vdf-size size-or-array)) (result-array (vdf size)))
  "gsl_sf_mathieu_a_array"
  ((minimum-order :int) ((+ minimum-order size -1) :int) (q :double)
   ((mpointer workspace) :pointer) ((foreign-pointer result-array) :pointer))
  :gsl-version (1 9)
  :outputs (result-array)
  :return (result-array)
  :documentation			; FDL
  "Compute a series of Mathieu characteristic values a_n(q) for n from
    minimum-order to minimum-order + size - 1 inclusive, where size
    is either the numerical value supplied in size-or-array, or the
    the length of the vector supplied there.")

(defmfun mathieu-b-array
    (q &optional
       size-or-array
       (minimum-order 0)
       (workspace
	(make-mathieu (+ minimum-order (vdf-size size-or-array)) q))
       &aux (size (vdf-size size-or-array)) (result-array (vdf size)))
  "gsl_sf_mathieu_b_array"
  ((minimum-order :int) ((+ minimum-order size -1) :int) (q :double)
   ((mpointer workspace) :pointer) ((foreign-pointer result-array) :pointer))
  :gsl-version (1 9)
  :outputs (result-array)
  :return (result-array)
  :documentation			; FDL
  "Compute a series of Mathieu characteristic values b_n(q) for n from
    minimum-order to minimum-order + size - 1 inclusive, where size
    is either the numerical value supplied in size-or-array, or the
    the length of the vector supplied there.")

;;;;****************************************************************************
;;;; Angular Mathieu functions
;;;;****************************************************************************

(defmfun mathieu-ce (n q x)
  "gsl_sf_mathieu_ce"
  ((n :int) (q :double) (x :double) (ret sf-result))
  :gsl-version (1 9)
  :documentation			; FDL
  "Compute the angular Mathieu functions ce_n(q,x).")

(defmfun mathieu-se (n q x)
  "gsl_sf_mathieu_se"
  ((n :int) (q :double) (x :double) (ret sf-result))
  :gsl-version (1 9)
  :documentation			; FDL
  "Compute the angular Mathieu functions se_n(q,x).")

(defmfun mathieu-ce-array
    (q x &optional
       size-or-array
       (minimum-order 0)
       (workspace
	(make-mathieu (+ minimum-order (vdf-size size-or-array)) q))
       &aux (size (vdf-size size-or-array)) (result-array (vdf size)))
  "gsl_sf_mathieu_ce_array"
  ((minimum-order :int) ((+ minimum-order size -1) :int)
   (q :double) (x :double)
   ((mpointer workspace) :pointer) ((foreign-pointer result-array) :pointer))
  :gsl-version (1 9)
  :outputs (result-array)
  :return (result-array)
  :documentation			; FDL
  "Compute a series of the angular Mathieu function ce_n(q) for n from
    minimum-order to minimum-order + size - 1 inclusive, where size
    is either the numerical value supplied in size-or-array, or the
    the length of the vector supplied there.")

(defmfun mathieu-se-array
    (q x &optional
       size-or-array
       (minimum-order 0)
       (workspace
	(make-mathieu (+ minimum-order (vdf-size size-or-array)) q))
       &aux (size (vdf-size size-or-array)) (result-array (vdf size)))
  "gsl_sf_mathieu_se_array"
  ((minimum-order :int) ((+ minimum-order size -1) :int)
   (q :double) (x :double)
   ((mpointer workspace) :pointer) ((foreign-pointer result-array) :pointer))
  :gsl-version (1 9)
  :outputs (result-array)
  :return (result-array)
  :documentation			; FDL
  "Compute a series of the angular Mathieu function se_n(q) for n from
    minimum-order to minimum-order + size - 1 inclusive, where size
    is either the numerical value supplied in size-or-array, or the
    the length of the vector supplied there.")

;;;;****************************************************************************
;;;; Radial Mathieu functions
;;;;****************************************************************************

(defmfun mathieu-Mc (j n q x)
  "gsl_sf_mathieu_Mc"
  ((j :int) (n :int) (q :double) (x :double) (ret sf-result))
  :gsl-version (1 9)
  :documentation			; FDL
  "Compute the radial j-th kind Mathieu functions Mc_n^{(j)}(q,x) of
   order n. The allowed values of j are 1 and 2. The functions for j =
   3,4 can be computed as M_n^{(3)} = M_n^{(1)} + iM_n^{(2)} and
   M_n^{(4)} = M_n^{(1)} - iM_n^{(2)}, where M_n^{(j)} = Mc_n^{(j)} or
   Ms_n^{(j)}.")

(defmfun mathieu-Ms (j n q x)
  "gsl_sf_mathieu_Ms"
  ((j :int) (n :int) (q :double) (x :double) (ret sf-result))
  :gsl-version (1 9)
  :documentation			; FDL
  "Compute the radial j-th kind Mathieu functions Ms_n^{(j)}(q,x) of order n.
   The allowed values of j are 1 and 2. The functions for j = 3,4 can
   be computed as M_n^{(3)} = M_n^{(1)} + iM_n^{(2)} and M_n^{(4)} =
   M_n^{(1)} - iM_n^{(2)}, where M_n^{(j)} = Mc_n^{(j)} or
   Ms_n^{(j)}.")

(defmfun mathieu-Mc-array
    (j q x &optional
       size-or-array
       (minimum-order 0)
       (workspace
	(make-mathieu (+ minimum-order (vdf-size size-or-array)) q))
       &aux (size (vdf-size size-or-array)) (result-array (vdf size)))
  "gsl_sf_mathieu_Mc_array"
  ((j :int) (minimum-order :int) ((+ minimum-order size) :int)
   (q :double) (x :double)
   ((mpointer workspace) :pointer) ((foreign-pointer result-array) :pointer))
  :gsl-version (1 9)
  :outputs (result-array)
  :return (result-array)
  :documentation			; FDL
  "Compute a series of the radial Mathieu function of kind j for n from
    minimum-order to minimum-order + size inclusive, where size
    is either the numerical value supplied in size-or-array, or the
    the length of the vector supplied there.")

(defmfun mathieu-Ms-array
    (j q x &optional
       size-or-array
       (minimum-order 0)
       (workspace
	(make-mathieu (+ minimum-order (vdf-size size-or-array)) q))
       &aux (size (vdf-size size-or-array)) (result-array (vdf size)))
  "gsl_sf_mathieu_Ms_array"
  ((j :int) (minimum-order :int) ((+ minimum-order size) :int)
   (q :double) (x :double)
   ((mpointer workspace) :pointer) ((foreign-pointer result-array) :pointer))
  :gsl-version (1 9)
  :outputs (result-array)
  :return (result-array)
  :documentation			; FDL
  "Compute a series of the radial Mathieu function of kind j for n from
    minimum-order to minimum-order + size inclusive, where size
    is either the numerical value supplied in size-or-array, or the
    the length of the vector supplied there.")

;;;;****************************************************************************
;;;; Examples
;;;;****************************************************************************

;;; Tests from gsl-1.11/specfunc/test_mathieu.c
(save-test mathieu
	   (mathieu-ce 0 0.0d0 0.0d0)
	   (mathieu-ce 0 0.0d0 (/ pi 2))
	   (mathieu-ce 0 5.0d0 0.0d0)
	   (mathieu-ce 0 5.0d0 (/ pi 2))
	   (mathieu-ce 0 10.0d0 0.0d0)
	   (mathieu-ce 0 10.0d0 (/ pi 2))
	   (mathieu-ce 0 15.0d0 0.0d0)
	   (mathieu-ce 0 15.0d0 (/ pi 2))
	   (mathieu-ce 0 20.0d0 0.0d0)
	   (mathieu-ce 0 20.0d0 (/ pi 2))
	   (mathieu-ce 0 25.0d0 0.0d0)
	   (mathieu-ce 0 25.0d0 (/ pi 2))
	   (mathieu-ce 1 0.0d0 0.0d0)
	   (mathieu-ce 1 5.0d0 0.0d0)
	   (mathieu-ce 1 10.0d0 0.0d0)
	   (mathieu-ce 1 15.0d0 0.0d0)
	   (mathieu-ce 1 20.0d0 0.0d0)
	   (mathieu-ce 1 25.0d0 0.0d0)
	   (mathieu-se 1 0.0d0 (/ pi 2))
	   (mathieu-se 1 5.0d0 (/ pi 2))
	   (mathieu-se 1 10.0d0 (/ pi 2))
	   (mathieu-se 1 15.0d0 (/ pi 2))
	   (mathieu-se 1 20.0d0 (/ pi 2))
	   (mathieu-se 1 25.0d0 (/ pi 2))
	   (mathieu-ce 2 0.0d0 0.0d0)
	   (mathieu-ce 2 0.0d0 (/ pi 2))
	   (mathieu-ce 2 5.0d0 0.0d0)
	   (mathieu-ce 2 5.0d0 (/ pi 2))
	   (mathieu-ce 2 10.0d0 0.0d0)
	   (mathieu-ce 2 10.0d0 (/ pi 2))
	   (mathieu-ce 2 15.0d0 0.0d0)
	   (mathieu-ce 2 15.0d0 (/ pi 2))
	   (mathieu-ce 2 20.0d0 0.0d0)
	   (mathieu-ce 2 20.0d0 (/ pi 2))
	   (mathieu-ce 2 25.0d0 0.0d0)
	   (mathieu-ce 2 25.0d0 (/ pi 2))
	   (mathieu-ce 5 0.0d0 0.0d0)
	   (mathieu-ce 5 5.0d0 0.0d0)
	   (mathieu-ce 5 10.0d0 0.0d0)
	   (mathieu-ce 5 15.0d0 0.0d0)
	   (mathieu-ce 5 20.0d0 0.0d0)
	   (mathieu-ce 5 25.0d0 0.0d0)
	   (mathieu-se 5 0.0d0 (/ pi 2))
	   (mathieu-se 5 5.0d0 (/ pi 2))
	   (mathieu-se 5 10.0d0 (/ pi 2))
	   (mathieu-se 5 15.0d0 (/ pi 2))
	   (mathieu-se 5 20.0d0 (/ pi 2))
	   (mathieu-se 5 25.0d0 (/ pi 2))
	   (mathieu-ce 10 0.0d0 0.0d0)
	   (mathieu-ce 10 0.0d0 (/ pi 2))
	   (mathieu-ce 10 5.0d0 0.0d0)
	   (mathieu-ce 10 5.0d0 (/ pi 2))
	   (mathieu-ce 10 10.0d0 0.0d0)
	   (mathieu-ce 10 10.0d0 (/ pi 2))
	   (mathieu-ce 10 15.0d0 0.0d0)
	   (mathieu-ce 10 15.0d0 (/ pi 2))
	   (mathieu-ce 10 20.0d0 0.0d0)
	   (mathieu-ce 10 20.0d0 (/ pi 2))
	   (mathieu-ce 10 25.0d0 0.0d0)
	   (mathieu-ce 10 25.0d0 (/ pi 2))
	   (mathieu-ce 15 0.0d0 0.0d0)
	   (mathieu-ce 15 5.0d0 0.0d0)
	   (mathieu-ce 15 10.0d0 0.0d0)
	   (mathieu-ce 15 15.0d0 0.0d0)
	   (mathieu-ce 15 20.0d0 0.0d0)
	   (mathieu-ce 15 25.0d0 0.0d0)
	   (mathieu-se 15 0.0d0 (/ pi 2))
	   (mathieu-se 15 5.0d0 (/ pi 2))
	   (mathieu-se 15 10.0d0 (/ pi 2))
	   (mathieu-se 15 15.0d0 (/ pi 2))
	   (mathieu-se 15 20.0d0 (/ pi 2))
	   (mathieu-se 15 25.0d0 (/ pi 2))
	   (grid:copy-to (mathieu-ce-array 0.0d0 (/ pi 2) 6))
	   (grid:copy-to (mathieu-ce-array 20.0d0 0.0d0 16))
	   (grid:copy-to (mathieu-se-array 20.0d0 (/ pi 2) 15 1)))
