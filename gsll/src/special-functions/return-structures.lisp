;; Structures returned by special functions.
;; Liam Healy, Mon Jan  1 2007 - 11:35
;; Time-stamp: <2010-06-29 22:15:22EDT return-structures.lisp>
;;
;; Copyright 2007, 2008, 2009 Liam M. Healy
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
;;;; Result from special functions
;;;;****************************************************************************

(defun val (sf-result &optional (type 'sf-result))
  (cffi:foreign-slot-value sf-result type 'val))

(defun err (sf-result &optional (type 'sf-result))
  (cffi:foreign-slot-value sf-result type 'err))

(defun e10 (sf-result)
  (cffi:foreign-slot-value sf-result 'sf-result-e10 'e10))

;;;;****************************************************************************
;;;; Array construction
;;;;****************************************************************************

(defparameter *default-sf-array-size* 5
  "The default size to make an array returned from a special function.")

(defun vdf (size-or-array)
  "Make or take a vector-double-float."
  (if (integerp size-or-array)
      (grid:make-foreign-array 'double-float :dimensions size-or-array)
      size-or-array))

(defun vdf-size (size-or-array)
  "Make or take a vector-double-float."
  (if (integerp size-or-array)
      size-or-array
      (size size-or-array)))
