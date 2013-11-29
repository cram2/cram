;; Generalized array
;; Liam Healy 2009-10-25 22:32:01EDT generalized-array.lisp
;; Time-stamp: <2010-06-19 22:40:21EDT array.lisp>
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

;;; These are method definitions for ordinary CL arrays

;;;;****************************************************************************
;;;; Properties of the grid
;;;;****************************************************************************

;;; Generic functions on grids that call array functions
(defmethod grid-rank ((grid array))
  (array-rank grid))

(defmethod dimensions ((grid array))
  (array-dimensions grid))

(defmethod element-type ((grid array))
  (array-element-type grid))

(defmethod affi ((grid array))
  (affi:make-affi (dimensions grid)))

;;;;****************************************************************************
;;;; Make grid
;;;;****************************************************************************

(defmethod make-grid-data
    ((type (eql 'array)) dimensions rest-spec
     &rest keys &key (initial-element nil initial-element-p)
     (initial-contents nil initial-contents-p))
  (let* ((element-type (top-spec-type rest-spec))
	 (array
	  (apply
	   'make-array
	   dimensions
	   :element-type element-type
	   (when (spec-scalar-p rest-spec)
	     (append
	      (when initial-element-p
		(list :initial-element (coerce-value initial-element element-type)))
	      (when initial-contents-p
		(list :initial-contents initial-contents)))))))
    array))

;;;;****************************************************************************
;;;; Element reference
;;;;****************************************************************************

(defmethod gref ((grid array) &rest indices)
  (apply 'aref grid indices))

(defmethod (setf gref) (value (grid array) &rest indices)
  (setf (apply #'aref grid indices) value))

(defmethod gref* ((grid array) linearized-index)
  (row-major-aref grid linearized-index))

(defmethod (setf gref*) (value (grid array) linearized-index)
  (setf (row-major-aref grid linearized-index) value))
