;; CFFI-Grovel definitions for unix systems.
;; Liam Healy 2009-06-06 09:32:30EDT monte-carlo-structs.lisp
;; Time-stamp: <2010-05-23 11:36:42EDT monte-carlo-structs.lisp>
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

#+linux
(define "_GNU_SOURCE")

;;; When installed through Mac Ports, GSL .h files will be found
;;; in /opt/local/include.
#+darwin
(cc-flags #.(gsl-config "--cflags"))

(include "gsl/gsl_monte_plain.h")

(cstruct plain-state "gsl_monte_plain_state"
  (dim "dim" :type sizet)
  (x "x" :type :pointer))

(include "gsl/gsl_monte_miser.h")

(cstruct miser-state "gsl_monte_miser_state"
  (min-calls "min_calls" :type sizet)
  (min-calls-per-bisection "min_calls_per_bisection" :type sizet)
  (dither "dither" :type :double)
  (estimate-frac "estimate_frac" :type :double)
  (alpha "alpha" :type :double)
  (dim "dim" :type sizet)
  (estimate-style "estimate_style" :type :int)
  (depth "depth" :type :int)
  (verbose "verbose" :type :int)
  (x "x" :type :pointer)
  (xmid "xmid" :type :pointer)
  (sigma-l "sigma_l" :type :pointer)
  (sigma-r "sigma_r" :type :pointer)
  (fmax-l "fmax_l" :type :pointer)
  (fmax-r "fmax_r" :type :pointer)
  (fmin-l "fmin_l" :type :pointer)
  (fmin-r "fmin_r" :type :pointer)
  (fsum-l "fsum_l" :type :pointer)
  (fsum-r "fsum_r" :type :pointer)
  (fsum2-l "fsum2_l" :type :pointer)
  (fsum2-r "fsum2_r" :type :pointer)
  (hits-l "hits_l" :type :pointer)
  (hits-r "hits_r" :type :pointer))

(include "gsl/gsl_monte_vegas.h")

(cstruct vegas-state "gsl_monte_vegas_state"
  ;; grid 
  (dim "dim" :type sizet)
  (bins-max "bins_max" :type sizet)
  (bins "bins" :type :uint)		       ; uint
  (boxes "boxes" :type :uint)		       ; these are both counted along the axes
  (xi "xi" :type :pointer)
  (xin "xin" :type :pointer)
  (delx "delx" :type :pointer)
  (weight "weight" :type :pointer)
  (vol "vol" :type :double)
  (x "x" :type :pointer)
  (bin "bin" :type :pointer)
  (box "box" :type :pointer)
  (d "d" :type :pointer)				; distribution
  ;; control variables
  (alpha "alpha" :type :double)
  (mode "mode" :type :int)
  (verbose "verbose" :type :int)
  (iterations "iterations" :type :uint)
  (stage "stage" :type :int))
