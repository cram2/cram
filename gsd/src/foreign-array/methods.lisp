;; Methods for grid functions
;; Liam Healy 2009-12-21 11:19:00EST methods.lisp
;; Time-stamp: <2010-07-15 18:12:46EDT methods.lisp>
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

(defmethod grid-rank ((object foreign-array))
  (length (dimensions object)))

;;;;****************************************************************************
;;;; Reference to elements
;;;;****************************************************************************

(defun linearized-index (object indices)
  (affi:calculate-index (affi object) indices))

(defmethod gref* ((object foreign-array) linearized-index)
  (let ((ptr
	 (cffi:mem-aref
	  (foreign-pointer object)
	  (cl-cffi (element-type object))
	  linearized-index)))
    (if (subtypep (element-type object) 'complex)
	;; The only reason to need FSBV here is for the conversion of
	;; foreign pointer (to complex) to native type; there isn't
	;; any call to libffi.
	#+fsbv
	(fsbv:object ptr (cl-cffi (element-type object)))
	#-fsbv ptr
	ptr)))

(defmethod gref ((object foreign-array) &rest indices)
  (gref* object (linearized-index object indices)))

(defmethod (setf gref*) (value (object foreign-array) linearized-index)
  ;; No recursion here, a foreign array is the end of the line.
  (symbol-macrolet
      ((ptr
	(cffi:mem-aref
	 (foreign-pointer object)
	 (cl-cffi (element-type object))
	 linearized-index)))
    (if (subtypep (element-type object) 'complex)
	#+fsbv
	(setf (fsbv:object ptr (cl-cffi (element-type object)))
	      value)
	#-fsbv
	(error "Cannot set element of complex array without FSBV.")
	(setf ptr value)))
  value)

(defmethod (setf gref) (value (object foreign-array) &rest indices)
  (setf (gref* object (linearized-index object indices))
	value))

(defmethod make-grid-data
    ((type (eql 'foreign-array)) dimensions rest-spec
     &rest keys &key initial-element
     &allow-other-keys)
  (let* ((element-type (spec-element-type rest-spec))
	 (array
	  (apply
	   'make-instance
	   ;; Make as the specific subclass of foreign-array,
	   ;; e.g. vector-double-float.
	   (data-class-name (length dimensions) element-type)
	   :dimensions dimensions
	   :element-type element-type
	   ;; Remove keywords to insure of non-redundancy
	   (alexandria::sans keys :element-type :dimensions))))
    array))

(defmethod copy ((object foreign-array)
		 &rest args
		 &key specification grid-type dimensions element-type
		 destination)
  (declare (ignorable specification grid-type dimensions element-type
		      destination))
  (apply 'copy-grid object args))
