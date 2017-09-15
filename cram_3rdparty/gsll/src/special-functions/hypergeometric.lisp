;; Hypergeometric function
;; Liam Healy, Fri Apr 28 2006 - 23:00
;; Time-stamp: <2011-11-24 22:46:31EST hypergeometric.lisp>
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

;; /usr/include/gsl/gsl_sf_hyperg.h

(in-package :gsl)

(defmfun hypergeometric-0F1 (c x)
  "gsl_sf_hyperg_0F1_e"
  ((c :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The hypergeometric function 0F1(c,x).")

(defgeneric hypergeometric-1F1 (m n x)
  (:documentation			; FDL
   "The confluent hypergeometric function 1F1(m,n,x) = M(m,n,x)."))

(defmfun hypergeometric-1F1 ((m integer) (n integer) x)
  "gsl_sf_hyperg_1F1_int_e"
  ((m :int) (n :int) (x :double) (ret (:pointer (:struct sf-result))))
  :definition :method
  :export t
  :documentation			; FDL
  "The confluent hypergeometric function 1F1(m,n,x) = M(m,n,x)
   for integer parameters m, n.")

(defmfun hypergeometric-1F1 ((a float) (b float) x)
  "gsl_sf_hyperg_1F1_e"
  ((a :double) (b :double) (x :double) (ret (:pointer (:struct sf-result))))
  :definition :method
  :documentation			; FDL
  "The confluent hypergeometric function
  1F1(a,b,x) = M(a,b,x) for general parameters a, b.")

(defgeneric hypergeometric-U (m n x)
  (:documentation			; FDL
   "The confluent hypergeometric function U(m,n,x)."))

(defmfun hypergeometric-U ((m integer) (n integer) x)
  "gsl_sf_hyperg_U_int_e"
  ((m :int) (n :int) (x :double) (ret (:pointer (:struct sf-result))))
  :definition :method
  :export t
  :documentation			; FDL
  "The confluent hypergeometric function U(m,n,x) for integer parameters m, n.")

(defmfun hypergeometric-U ((a float) (b float) x)
  "gsl_sf_hyperg_U_e"
  ((a :double) (b :double) (x :double) (ret (:pointer (:struct sf-result))))
  :definition :method
  :documentation "The confluent hypergeometric function U(a,b,x).")

(defgeneric hypergeometric-U-e10 (m n x)
  (:documentation			; FDL
   "The confluent hypergeometric function
   U(m,n,x) that returns a result with extended range."))

(defmfun hypergeometric-U-e10 ((m integer) (n integer) x)
  "gsl_sf_hyperg_U_int_e10_e"
  ((m :int) (n :int) (x :double) (ret (:pointer (:struct sf-result-e10))))
  :definition :method
  :export t
  :documentation			; FDL
  "The confluent hypergeometric function
   U(m,n,x) for integer parameters m, n that returns a
   result with extended range.")

(defmfun hypergeometric-U-e10 ((a float) (b float) x)
  "gsl_sf_hyperg_U_e10_e"
  ((a :double) (b :double) (x :double) (ret (:pointer (:struct sf-result-e10))))
  :definition :method
  :documentation			; FDL
  "The confluent hypergeometric function
  U(a,b,x) using that returns a result with extended range.")

(defmfun hypergeometric-2F1 (a b c x)
  "gsl_sf_hyperg_2F1_e"
  ((a :double) (b :double) (c :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Gauss hypergeometric function
  2F1(a,b,c,x) for |x| < 1. If the arguments
  (a,b,c,x) are too close to a singularity then the function can
  signal the error 'exceeded-maximum-iterations when the series
  approximation converges too slowly.  This occurs in the region of
  x=1, c - a - b = m for integer m.")

(defmfun hypergeometric-2F1-conj (a c x)
  "gsl_sf_hyperg_2F1_conj_e"
  (((realpart a) :double) ((imagpart a) :double)
   (c :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The Gauss hypergeometric function 2F1(a, a*, c, x) with complex parameters 
  for |x| < 1.")

(defmfun hypergeometric-2F1-renorm (a b c x)
  "gsl_sf_hyperg_2F1_renorm_e"
  ((a :double) (b :double) (c :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The renormalized Gauss hypergeometric function
  2F1(a,b,c,x) / Gamma(c) for |x| < 1.")

(defmfun hypergeometric-2F1-conj-renorm (a c x)
  "gsl_sf_hyperg_2F1_conj_renorm_e"
  (((realpart a) :double) ((imagpart a) :double)
   (c :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation			; FDL
  "The renormalized Gauss hypergeometric function
  2F1(a, a*, c, x) / Gamma(c) for |x| < 1.")

(defmfun hypergeometric-2F0 (a b x)
  "gsl_sf_hyperg_2F0_e"
  ((a :double) (b :double) (x :double) (ret (:pointer (:struct sf-result))))
  :documentation 			; FDL
  "The hypergeometric function 
  2F0(a,b,x).  The series representation
  is a divergent hypergeometric series.  However, for x < 0 we
  have 2F0(a,b,x) = (-1/x)^a U(a,1+a-b,-1/x)")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test hypergeometric
  (hypergeometric-0f1 0.5d0 1.0d0)
  (hypergeometric-1F1 2 1 1.0d0)
  (hypergeometric-1F1 2.0d0 1.0d0 1.0d0)
  (hypergeometric-U 2.0d0 1.0d0 1.0d0)
  (hypergeometric-U 2 1 1.0d0)
  (hypergeometric-U-e10 2.0d0 1.0d0 1.0d0)
  (hypergeometric-U-e10 2 1 1.0d0)
  (hypergeometric-2F1 1.0d0 1.2d0 1.0d0 0.5d0)
  (hypergeometric-2F1-conj #C(1.0d0 0.5d0) 0.5d0 0.6d0)
  (hypergeometric-2F1-conj-renorm #C(1.0d0 0.5d0) 0.5d0 0.6d0)
  (hypergeometric-2F1-renorm 1.0d0 1.2d0 1.0d0 0.5d0)
  (hypergeometric-2F0 1.0d0 2.0d0 -20.0d0))

