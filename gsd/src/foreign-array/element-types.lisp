;; Mapping of element type names
;; Liam Healy 2008-04-13 11:22:46EDT element-types.lisp
;; Time-stamp: <2010-06-29 21:17:32EDT element-types.lisp>
;;
;; Copyright 2008, 2009, 2010 Liam M. Healy
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
;; $Id$

(in-package :grid)

(export '(*array-element-types* *array-element-types-no-complex*
	  *float-complex-types* *float-types* *complex-types*
	  *double-types* element-types element-size))

;;;;****************************************************************************
;;;; Common element type groups for generic functions
;;;;****************************************************************************

(defparameter *array-element-types*
  (remove-duplicates (all-types *cstd-cl-type-mapping* t) :test 'equal)
  "All the array element types supported.")

(defparameter *array-element-types-no-complex*
  (remove-if (lambda (tp) (subtypep tp 'complex)) *array-element-types*)
  "All the array element types supported except for complex types.")

(defparameter *float-complex-types*
  (remove-if (lambda (tp) (subtypep tp 'integer)) *array-element-types*)
  "All the float or complex array element types supported.")

(defparameter *float-types*
  (remove-if-not (lambda (tp) (subtypep tp 'float)) *array-element-types*)
  "All the float array element types.")

(defparameter *complex-types*
  (remove-if-not (lambda (tp) (subtypep tp 'complex)) *array-element-types*)
  "All the supported complex array element types.")

(defparameter *double-types*
  (list 'double-float '(complex double-float))
  "All the supported double float element types.")

(defun element-types (element-types)
  "All the element types available of a given category."
  (case element-types
    ((nil t) *array-element-types*)
    (:no-complex *array-element-types-no-complex*)
    (:float *float-types*)
    (:complex *complex-types*)
    (:float-complex *float-complex-types*)
    (:doubles *double-types*)
    (t element-types)))

(defun element-size (object)
  "The size of each element as stored in C."
  (cffi:foreign-type-size (cl-cffi (element-type object))))
