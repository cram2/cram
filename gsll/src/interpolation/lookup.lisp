;; Index lookup and acceleration
;; Liam Healy, Sun Nov  4 2007 - 18:09
;; Time-stamp: <2012-01-13 12:01:25EST lookup.lisp>
;;
;; Copyright 2007, 2008, 2009, 2011 Liam M. Healy
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

(defmobject acceleration "gsl_interp_accel"
  ()
  "acceleration for interpolation"
  :documentation			; FDL
  "Make an accelerator object, which is a
   kind of iterator for interpolation lookups.  It tracks the state of
   lookups, thus allowing for application of various acceleration
   strategies.")

(defmfun interpolation-search (x-array x low-index high-index)
  "gsl_interp_bsearch"
  ((x-array :pointer) (x :double) (low-index :sizet) (high-index :sizet))
  :c-return :sizet
  :documentation			; FDL
  "Find the index i of the array x-array such
   that x-array[i] <= x < x-array[i+1].  The index is searched for
   in the range [low-index, high-index].")

(defmfun accelerated-interpolation-search (x-array x acceleration)
  "gsl_interp_accel_find"
  (((mpointer acceleration) :pointer) (x-array :pointer) (x :double))
  :c-return :sizet
  :documentation			; FDL
  "Search the data array x-array of size, using the given acceleration.
   This is how lookups are performed during evaluation of an interpolation.  The
   function returns an index i such that x_array[i] <= x < x_array[i+1]}.")
