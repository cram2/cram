;; Grid methods for physical dimension quantities
;; Liam Healy 2011-01-17 18:36:54EST grid.lisp
;; Time-stamp: <2013-11-23 09:50:59EST grid.lisp>

;; Copyright 2011, 2012, 2013 Liam M. Healy
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

;;; Physical dimension quantities with a magnitude that is a scalar
;;; (not a grid).

(in-package :antik)

;;;;****************************************************************************
;;;; make-pq-grid
;;;;****************************************************************************

(defun make-pq-grid-NEW
    (unit-expressions same-units &key (initial-element nil initial-element-p)
     (initial-contents nil initial-contents-p)
     specification)
  "Construct a pdq grid given one of:
   1) initial-contents of pdqs and specification
   2) initial-contents of magnitudes, unit-expressions,
       and optionally specification (else same as magnitudes')
   3) initial-element being a pdq, and specification
   4) specification and optionally same-units."
  (let ((spec (or specification (rest (grid:specification initial-contents)))))
    (make-pq-object
     (apply #'grid::make-grid		; the magnitude
	    spec
	    (cond (initial-element-p
		   (list :initial-element (pq-magnitude initial-element)))
		  (initial-contents-p
		   (list :initial-contents (map-pq #'pq-magnitude initial-contents)))
		  (t nil)))
     (or same-units			; the units
	 (apply
	  #'make-array
	  (grid::specification-dimensions spec)
	  :element-type t
	  (cond 
	    (initial-contents-p
	     (list :initial-contents (map-pq #'pq-dimension initial-contents)))
	    (unit-expressions (list :initial-contents unit-expressions))
	    (t (list :initial-element nil)))))
     (not (not same-units)))))

(defun make-pq-grid (magnitude unit-expressions)
  "Make a physical dimension quantity out of a grid where the elements may have different units.  This may modify the contents of the magnitude grid."
  ;; We make a scalar pdq out of each element in order to compute the
  ;; SI value of the magnitude before making the pdq object.
  (loop with unit-array = (make-array (grid:dimensions magnitude))
	and ue = (if (grid:gridp unit-expressions)
		     unit-expressions
		     (make-array
		      (grid:dimensions unit-expressions)
		      :element-type t
		      :initial-contents unit-expressions))
	for i below (grid:total-size magnitude)
	for pq-elt = (make-pq-ue (grid:aref* magnitude i) (grid:aref* ue i))
	do
     (setf (grid:aref* magnitude i) (pq-magnitude pq-elt)
	   (grid:aref* unit-array i) (pq-dimension pq-elt))
	finally (return (make-pq-object magnitude unit-array nil))))

(defun pq-gridp (object)
  (and (physical-quantity-p object)
       (grid:gridp (antik::pq-magnitude object))
       (list 'physical-quantity (grid:gridp (antik::pq-magnitude object)))))

(setf grid::*additional-gridp-test* #'pq-gridp)

;;;;****************************************************************************
;;;; Grid methods
;;;;****************************************************************************

(defun map-pq (function object &optional flatten)
  "Map the function of a pdq onto the nested lists of pdqs, optionally flattening."
  (etypecase object
    ((or number physical-quantity) (funcall function object))
    (list (funcall
	   (if (and flatten (listp (first object)))
	       #'alexandria:mappend
	       #'mapcar)
	   (lambda (ob) (map-pq function ob flatten)) object))))

(defmethod grid::make-grid-data
  ((type (eql 'physical-quantity)) dimensions rest-spec
   &key
   (initial-element nil initial-element-p)
   (initial-contents nil initial-contents-p))
  (declare (ignore dimensions))
  (let ((same-units
	 (cond
	   (initial-element-p (pq-dimension initial-element))
	   (initial-contents-p (equal-dimensions initial-contents)))))
    (make-pq-object
     (apply #'grid::make-grid
	    rest-spec
	    (cond (initial-element-p
		   (list :initial-element (pq-magnitude initial-element)))
		  (initial-contents-p
		   (list :initial-contents (map-pq #'pq-magnitude initial-contents)))
		  (t nil)))
     (or same-units
	 (apply
	  #'make-array
	  (grid::specification-dimensions rest-spec)
	  :element-type t
	  (if initial-contents-p
	      (list
	       :initial-contents
	       (map-pq #'pq-dimension initial-contents))
	      (list :initial-element nil))))
     (not (not same-units)))))

(defmethod grid::make-grid-data :around
    (type dimensions rest-spec &rest keys &key initial-element initial-contents)
  ;; Check if a grid is being made with pdq initializing but not speced as pdq
  (if (eq type 'physical-quantity)
      (call-next-method)
      (if (or (and initial-contents
		   (some (lambda (x) (physical-quantity-p x))
			 (alexandria:flatten initial-contents)))
	      (physical-quantity-p initial-element))
	  (apply #'grid::make-grid-data
		 'physical-quantity
		 nil
		 (apply 'grid::make-specification type dimensions rest-spec)
		 keys)
	  (call-next-method))))

(defmethod grid::affi ((grid physical-quantity))
  (when (grid:gridp grid)
    (grid::affi (pq-magnitude grid))))

(defmethod grid:aref ((object physical-quantity) &rest indices)
  (assert (grid:gridp (pq-magnitude object)) (object))
  (make-pq-object
   (apply #'grid:aref (pq-magnitude object) indices)
   (if (scalar-dimension object)
       (pq-dimension object)
       (apply #'grid:aref (pq-dimension object) indices))
   t))

(defmethod grid:aref* ((object physical-quantity) index)
  (assert (grid:gridp (pq-magnitude object)) (object))
  (make-pq-object
   (grid:aref* (pq-magnitude object) index)
   (if (scalar-dimension object)
       (pq-dimension object)
       (grid:aref* (pq-dimension object) index))
   t))

(defmethod (setf grid:aref) (value (object physical-quantity) &rest indices)
  (assert (grid:gridp (pq-magnitude object)) (object))
  (assert (or (and (scalar-dimension object) (equal-dimension value object nil))
	      (physical-quantity-p value)))
  ;; Value should be a scalar; it can be a pdq or not.  If it is not a
  ;; pdq, then object should have scalar-units = t so that the
  ;; appropriate units may be adopted.
  (if (and (scalar-dimension object) (equal-dimension value object nil))
      (apply #'(setf grid:aref) (pq-magnitude value) (pq-magnitude object) indices)
      (progn
	(apply #'(setf grid:aref) (pq-magnitude value) (pq-magnitude object) indices)
	(apply #'(setf grid:aref) (pq-dimension value) (pq-dimension object) indices)))
  value)

(defmethod (setf grid:aref*) (value (object physical-quantity) index)
  (assert (grid:gridp (pq-magnitude object)) (object))
  (assert (or (and (scalar-dimension object) (equal-dimension value object nil))
	      (and (zerop value) antik::*zero-is-dimensionless*)
	      (physical-quantity-p value))
	  (value)
	  "The element cannot be set to ~a because it ~
           is not a physical dimension quantity and the array does not have ~
           scalar units." value)
  ;; Value should be a scalar; it can be a pdq or not.  If it is not a
  ;; pdq, then object should have scalar-units = t so that the
  ;; appropriate units may be adopted.
  (if (and (scalar-dimension object) (equal-dimension value object nil))
      (setf (grid:aref* (pq-magnitude object) index) value)
      (progn
	(setf (grid:aref* (pq-magnitude object) index) (pq-magnitude value))
	(setf (grid:aref* (pq-dimension object) index) (pq-dimension value))))
  value)

(defmethod grid:dimensions ((object physical-quantity))
	   (grid:dimensions (pq-magnitude object)))

(defmethod grid:rank ((object physical-quantity))
	   (grid:rank (pq-magnitude object)))

(defmethod grid:total-size ((object physical-quantity))
	   (grid:total-size (pq-magnitude object)))

(defmethod grid:element-type ((object physical-quantity))
	   (grid:element-type (pq-magnitude object)))

(defun grid::pdq-grid-specification (specification)
  "Make a specification for a PDQ grid from the specification of its magnitude."
  (cons 'physical-quantity specification))

(defmethod grid:specification ((object physical-quantity))
	   (grid::pdq-grid-specification
		 (grid:specification (pq-magnitude object))))

(defmethod grid::print-grid-readably
    ((object physical-quantity) contents-writer stream)
  (grid::print-grid-readably
   (pq-magnitude object)
   contents-writer
   stream))
