;; Conversion of numbers C->CL
;; Liam Healy, Sun May 28 2006 - 22:04
;; Time-stamp: <2010-06-19 18:47:04EDT number-conversion.lisp>
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

(in-package :grid)

(export '(complex-to-cl))

;;;;****************************************************************************
;;;; Built-in numerical types
;;;;****************************************************************************

(export '(dcref))

(defmacro dcref (double &optional (index 0))
  "Reference C double(s)."
  `(cffi:mem-aref ,double :double ,index))

;;;;****************************************************************************
;;;; Complex numbers
;;;;****************************************************************************

;;; Complex struct is defined in init/complex-types.lisp.
(defun complex-to-cl
    (complex &optional (index 0) (complex-type 'complex-double-c))
  "Make a CL complex number from the pointer to a complex struct or
   an array of complex structs and an index into the array." 
  (let ((carr (cffi:foreign-slot-value
	       (cffi:inc-pointer
		complex
		(* index (cffi:foreign-type-size complex-type)))
	       complex-type 'dat)))
    (complex (dcref carr 0)
	     (dcref carr 1))))
	
