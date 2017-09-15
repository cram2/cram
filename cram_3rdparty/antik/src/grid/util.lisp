;; Utility
;; Liam Healy 2009-11-28 22:44:13EST util.lisp
;; Time-stamp: <2013-11-27 10:45:03EST util.lisp>
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
  "The second dimension of the object."
  (second (dimensions object)))

;;;;****************************************************************************
;;;; Numerical values
;;;;****************************************************************************

(defparameter *default-numerical-value* 0)

(defun coerce-value (value type)
  "Coerce the numeric value to the type.  If value is not numeric, return it."
  (if type
      (if (listp value)
	  (mapcar (alexandria:rcurry #'coerce-value type) value)
	  (if (subtypep type 'number)
	      (if (or (null value) (typep value 'number))
		  (coerce (or value *default-numerical-value*) type)
		  value)
	      (coerce value type)))
      value))


;;;;****************************************************************************
;;;; Readable output
;;;;****************************************************************************

(defgeneric print-grid-readably (object contents-writer stream)
  (:documentation "Print the grid readably. The contents-writer is a function that is called on the stream and writes the contents of the grid.")
  (:method ((object t) contents-writer stream)
    (declare (ignore object))
    (funcall contents-writer stream)))

(defmethod print-grid-readably ((object array) contents-writer stream)
  (format stream (if (= (length (dimensions object)) 2) "#2A" "#"))
  (funcall contents-writer stream))

