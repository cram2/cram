;; Structures returned by special functions.
;; Liam Healy, Mon Jan  1 2007 - 11:35
;; Time-stamp: <2012-01-13 12:01:15EST return-structures.lisp>
;;
;; Copyright 2007, 2008, 2009, 2010, 2011, 2012 Liam M. Healy
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

;;; Define methods for cffi:translate-from-foreign for translate-from-foreign
;;; that return the slots as successive values.

(cffi:define-translation-method (pointer sf-result :from)
  (let ((plist (call-next-method)))
    (values (getf plist 'val) (getf plist 'err))))

(cffi:define-translation-method (pointer sf-result-e10 :from)
  (let ((plist (call-next-method)))
    (values (getf plist 'val) (getf plist 'e10) (getf plist 'err))))

(defun complex-with-error (real-sfr imaginary-sfr)
  "Return two complex numbers, the value and the error."
  (metabang-bind:bind
      (((:values treal-val treal-err)
	(cffi:convert-from-foreign real-sfr '(:struct sf-result)))
       ((:values timag-val timag-err)
	(cffi:convert-from-foreign imaginary-sfr '(:struct sf-result))))
    (values
     (complex treal-val timag-val)
     (complex treal-err timag-err))))

(defun values-with-errors (&rest values-sfr)
  "Return numbers as values and errors."
  (loop for vs in values-sfr
	for tvs
	  =  (multiple-value-list
	      (cffi:convert-from-foreign vs '(:struct sf-result)))
	collect (first tvs) into values
	collect (second tvs) into errors
	finally (return (values-list (append values errors)))))

;;;;****************************************************************************
;;;; Array construction
;;;;****************************************************************************

(defparameter *default-sf-array-size* 5
  "The default size to make an array returned from a special function.")

(defun vdf (size-or-array &optional (type 'double-float))
  "Make or take a vector."
  (if (integerp size-or-array)
      (grid:make-foreign-array
       (if (eq type ':sizet)
	   #+int64 '(unsigned-byte 64)
	   #+int32 '(unsigned-byte 32)
	   type)
       :dimensions size-or-array)
      size-or-array))

(defun vdf-size (size-or-array)
  "Size of vector made with vfd."
  (if (integerp size-or-array)
      size-or-array
      (size size-or-array)))
