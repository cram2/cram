;; Utility
;; Liam Healy 2009-11-28 22:44:13EST util.lisp
;; Time-stamp: <2010-07-20 10:53:57EDT util.lisp>
;;
;; Copyright 2009 Liam M. Healy
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

(in-package :grid)

;;;;****************************************************************************
;;;; List modification
;;;;****************************************************************************

(defun remove-position (sequence position &optional (result-type 'list))
  "Remove the element at position."
  (concatenate
   result-type
   (subseq sequence 0 position)
   (when (< (1+ position) (length sequence))
     (subseq sequence (1+ position)))))

(defun insert-position (value sequence position &optional (result-type 'list))
  "Insert the element at position."
  (concatenate
   result-type
   (subseq sequence 0 position)
   (list value)
   (when (< position (length sequence))
     (subseq sequence position))))

(defun set-position (list position value)
  "Set the element at position to the value, and return the list."
  (let ((lst (copy-list list)))
    (setf (nth position lst) value)
    lst))

(defun one-non-zero-list (length position value)
  "Make list of the specified length that is all zeros except in the
   position specified, which has the value given."
  (let ((list (make-list length :initial-element 0)))
    (setf (nth position list) value)
    list))

(defun sequential-list (length)
  "Make a list of the given length with element equal to the index."
  (loop for i below length collect i))

;;;;****************************************************************************
;;;; Total size
;;;;****************************************************************************

(defun total-size-from-dimension (dimensions)
  (reduce #'* dimensions))

;;;;****************************************************************************
;;;; Vector and matrix reference
;;;;****************************************************************************

(export '(dim0 dim1))

(defun dim0 (object)
  "The first dimension of the object."
  (first (dimensions object)))

(defun dim1 (object)
  "The first dimension of the object."
  (second (dimensions object)))

;;;;****************************************************************************
;;;; Numerical values
;;;;****************************************************************************

(defparameter *default-numerical-value* 0)

(defun coerce-value (value type)
  "Coerce the value to the type."
  (if type
      (if (and (null value) (subtypep type 'number))
	  (coerce *default-numerical-value* type)
	  (coerce value type))
      value))

;;;;****************************************************************************
;;;; Grid type
;;;;****************************************************************************

(export '(*grid-types* *default-grid-type* gridp))

(defparameter *grid-types* '(array))
(defparameter *default-grid-type* 'array)

(defun gridp (object)
  "Object is a grid; type is returned."
  (some (lambda (type) (typep object type)) *grid-types*))
