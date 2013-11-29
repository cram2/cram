;; Airy functions
;; Liam Healy, Fri Mar 17 2006 - 18:41
;; Time-stamp: <2009-12-27 10:10:07EST airy.lisp>
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
;;;; Airy functions
;;;;****************************************************************************

(defmfun airy-Ai (x &optional (mode :double))
    "gsl_sf_airy_Ai_e" ((x :double) (mode sf-mode) (ret sf-result))
  :documentation			; FDL
  "The Airy function Ai(x).")

(defmfun airy-Bi (x &optional (mode :double))
  "gsl_sf_airy_Bi_e" ((x :double) (mode sf-mode) (ret sf-result))
  :documentation			; FDL
  "The Airy function Bi(x).")

(defmfun airy-Ai-scaled (x &optional (mode :double))
  "gsl_sf_airy_Ai_scaled_e" ((x :double) (mode sf-mode) (ret sf-result))
  :documentation			; FDL
  "The scaled Airy function S_A(x) Ai(x).
   For x>0 the scaling factor S_A(x) is \exp(+(2/3) x^(3/2)),
   and is 1 for x<0.")

(defmfun airy-Bi-scaled (x &optional (mode :double))
  "gsl_sf_airy_Bi_scaled_e" ((x :double) (mode sf-mode) (ret sf-result))
  :documentation			; FDL
  "The scaled Airy function S_B(x) Bi(x).
   For x>0 the scaling factor S_B(x) is exp(-(2/3) x^(3/2)),
   and is 1 for x<0.")

(defmfun airy-Ai-deriv (x &optional (mode :double))
  "gsl_sf_airy_Ai_deriv_e" ((x :double) (mode sf-mode) (ret sf-result))
  :documentation			; FDL
  "The Airy function derivative Ai'(x).")

(defmfun airy-Bi-deriv (x &optional (mode :double))
  "gsl_sf_airy_Bi_deriv_e" ((x :double) (mode sf-mode) (ret sf-result))
  :documentation "The Airy function derivative Bi'(x).")

(defmfun airy-Ai-deriv-scaled (x &optional (mode :double))
  "gsl_sf_airy_Ai_deriv_scaled_e" ((x :double) (mode sf-mode) (ret sf-result))
  :documentation			; FDL
  "The scaled Airy function derivative S_A(x) Ai'(x).
  For x>0 the scaling factor S_A(x) is exp(+(2/3) x^(3/2)),
  and is 1 for x<0.")

(defmfun airy-Bi-deriv-scaled (x &optional (mode :double))
  "gsl_sf_airy_Bi_deriv_scaled_e" ((x :double) (mode sf-mode) (ret sf-result))
  :documentation			; FDL
  "The scaled Airy function derivative S_B(x) Bi'(x).
   For x>0 the scaling factor S_B(x) is
   exp(-(2/3) x^(3/2)), and is 1 for x<0.")

(defmfun airy-zero-Ai (s)
  "gsl_sf_airy_zero_Ai_e" ((s sizet) (ret sf-result))
  :documentation			; FDL
  "The location of the s-th zero of the Airy function Ai(x).")

(defmfun airy-zero-Bi (s)
  "gsl_sf_airy_zero_Bi_e" ((s sizet) (ret sf-result))
  :documentation			; FDL
  "The location of the s-th zero of the Airy function Bi(x).")

(defmfun airy-zero-Ai-deriv (s)
  "gsl_sf_airy_zero_Ai_deriv_e" ((s sizet) (ret sf-result))
  :documentation			; FDL
  "The location of the s-th zero of the Airy function derivative Ai'(x).")

(defmfun airy-zero-Bi-deriv (s)
  "gsl_sf_airy_zero_Bi_deriv_e" ((s sizet) (ret sf-result))
  :documentation			; FDL
  "The location of the s-th zero of the Airy function derivative Bi'(x).")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

;; GSL tests from gsl-1.11/specfunc/test_airy.c
(save-test airy
  (airy-ai -500.0d0)
  (airy-ai -5.0d0)
  (airy-ai -0.3000000000000094d0)
  (airy-ai 0.6999999999999907d0)
  (airy-ai 1.649999999999991d0)
  (airy-ai 2.54999999999999d0)
  (airy-ai 3.499999999999987d0)
  (airy-ai 5.39999999999998d0)
  (airy-ai-scaled -5.0d0)
  (airy-ai-scaled 0.6999999999999907d0)
  (airy-ai-scaled 1.649999999999991d0)
  (airy-ai-scaled 2.54999999999999d0)
  (airy-ai-scaled 3.499999999999987d0)
  (airy-ai-scaled 5.39999999999998d0)
  (airy-bi -500.0d0)
  (airy-bi -5.0d0)
  (airy-bi 0.6999999999999907d0)
  (airy-bi 1.649999999999991d0)
  (airy-bi 2.54999999999999d0)
  (airy-bi 3.499999999999987d0)
  (airy-bi 5.39999999999998d0)
  (airy-bi-scaled -5.0d0)
  (airy-bi-scaled 0.6999999999999907d0)
  (airy-bi-scaled 1.649999999999991d0)
  (airy-bi-scaled 2.54999999999999d0)
  (airy-bi-scaled 3.499999999999987d0)
  (airy-bi-scaled 5.39999999999998d0)
  (airy-ai-deriv -5.0d0)
  (airy-ai-deriv -0.5500000000000094d0)
  (airy-ai-deriv 0.4999999999999906d0)
  (airy-ai-deriv 1.899999999999992d0)
  (airy-ai-deriv 3.249999999999988d0)
  (airy-ai-deriv 5.199999999999981d0)
  (airy-ai-deriv-scaled -5.0d0)
  (airy-ai-deriv-scaled 0.5499999999999906d0)
  (airy-ai-deriv-scaled 1.499999999999991d0)
  (airy-ai-deriv-scaled 2.49999999999999d0)
  (airy-ai-deriv-scaled 3.649999999999986d0)
  (airy-ai-deriv-scaled 6.299999999999977d0)
  (airy-bi-deriv -5.0d0)
  (airy-bi-deriv -0.5500000000000094d0)
  (airy-bi-deriv 0.4999999999999906d0)
  (airy-bi-deriv 1.899999999999992d0)
  (airy-bi-deriv 3.249999999999988d0)
  (airy-bi-deriv 5.199999999999981d0)
  (airy-bi-deriv-scaled -5.0d0)
  (airy-bi-deriv-scaled 0.5499999999999906d0)
  (airy-bi-deriv-scaled 1.499999999999991d0)
  (airy-bi-deriv-scaled 2.49999999999999d0)
  (airy-bi-deriv-scaled 3.649999999999986d0)
  (airy-bi-deriv-scaled 6.299999999999977d0)
  (airy-zero-ai 2)
  (airy-zero-ai 50)
  (airy-zero-ai 100)
  (airy-zero-ai 110)
  (airy-zero-bi 2)
  (airy-zero-bi 50)
  (airy-zero-bi 100)
  (airy-zero-bi 110)
  (airy-zero-bi 111)
  (airy-zero-bi 200)
  (airy-zero-ai-deriv 2)
  (airy-zero-ai-deriv 50)
  (airy-zero-ai-deriv 100)
  (airy-zero-ai-deriv 110)
  (airy-zero-ai-deriv 1000)
  (airy-zero-bi-deriv 2)
  (airy-zero-bi-deriv 50)
  (airy-zero-bi-deriv 100)
  (airy-zero-bi-deriv 110)
  (airy-zero-bi-deriv 111)
  (airy-zero-bi-deriv 200)
  (airy-zero-bi-deriv 1000))

#|
;;; Mathematica results
In[4]:= AiryAi[2.5]
Out[4]= 0.01572592338047049
In[5]:= AiryBi[2.5]
Out[5]= 6.481660738460579
In[6]:= AiryAiPrime[2.5]
Out[6]= -0.02625088103590323
In[7]:= AiryBiPrime[2.5]
Out[7]= 9.4214233173343
|#
