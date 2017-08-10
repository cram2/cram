;; CFFI-Grovel definitions for unix systems.
;; Liam Healy 2009-06-06 09:36:29EDT array-structs.lisp
;; Time-stamp: <2012-01-13 12:01:37EST array-structs.lisp>
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

(include "gsl/gsl_block_double.h")

(cstruct gsl-block-c "gsl_block"
  (size "size" :type :sizet)
  (data "data" :type :pointer))

(include "gsl/gsl_vector_double.h")

(cstruct gsl-vector-c "gsl_vector"
  (size "size" :type :sizet)
  (stride "stride" :type :sizet)
  (data "data" :type :pointer)
  (block "block" :type :pointer)
  (owner "owner" :type :int))

(include "gsl/gsl_matrix_double.h")

(cstruct gsl-matrix-c "gsl_matrix"
  (size0 "size1" :type :sizet)
  (size1 "size2" :type :sizet)
  (tda "tda" :type :sizet)
  (data "data" :type :pointer)
  (block "block" :type :pointer)
  (owner "owner" :type :int))

(include "gsl/gsl_permutation.h")

(cstruct gsl-permutation-c "gsl_permutation"
  (size "size" :type :sizet)
  (data "data" :type :pointer))

(include "gsl/gsl_combination.h")

(cstruct gsl-combination-c "gsl_combination"
  (range "n" :type :sizet)
  (size "k" :type :sizet)
  (data "data" :type :pointer))
