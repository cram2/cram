;; Physical dimension quantity with a magnitude that is a scalar
;; Liam Healy 2011-01-01 12:05:37EST grid.lisp
;; Time-stamp: <2015-12-17 14:59:02EST scalar.lisp>

;; Copyright 2011, 2013, 2015 Liam M. Healy
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

(in-package :antik)

(export '(physical-quantity physical-quantity-p ; already exported package def
	  *zero-is-dimensionless* make-pq-zero make-pq))

;;;;****************************************************************************
;;;; The class and functions to make instances
;;;;****************************************************************************

(defgeneric pq-dimension (object)
  (:documentation "Dimel for the physical dimension quantity.")
  (:method ((object number))
	   (dimension 'dimensionless)))

(defgeneric pq-magnitude (object)
  (:documentation "Numerical magnitude of a physical dimension quantity.")
  (:method ((object number))
	   object))

(defgeneric scalar-dimension (object)
  (:documentation "Physical dimensions for object are a scalar.")
  (:method ((object t))
	   (declare (ignore object))
	   t))

(defclass physical-quantity ()
  ((magnitude :initarg :magnitude :reader pq-magnitude)
   (dimension :initarg :dimension :reader pq-dimension
	      :type (or (satisfies dimelp) array))
   (scalar-dimension :initarg :scalar-dimension :reader scalar-dimension))
  (:documentation "A quantity with a dimension and possibly a unit,
  e.g. length or meter."))

(defun physical-quantity-p (x) (typep x 'antik:physical-quantity))

(defun make-pq-object (mag dimel scalar-dimension)
  #+debug
  (when (physical-quantity-p mag)
    (error "Magnitude ~a can't be a PDQ." mag))
  #+debug
  (assert (if scalar-dimension
	      (dimelp dimel)
	      (when (arrayp dimel)
		;; If 'dimel is a list but not an array for
		;; scalar-dimension=nil, then this will give an error
		;; that says it's not a dimel, which is wrong, but you
		;; get the idea.
		(if (vectorp dimel)
		    (or (every 'dimelp dimel) (notany 'identity dimel))
		    t)))     ; only check vectors for non-scalar units
	  (dimel)
	  "~a is not a dimel" dimel)
  (if (or (and *zero-is-dimensionless* (zerop mag) (not (grid:gridp mag)))
	  (equal dimel (dimension 'dimensionless)))
      mag
      (make-instance
       'physical-quantity
       :magnitude mag :dimension dimel :scalar-dimension scalar-dimension)))

(defparameter *zero-is-dimensionless* t
  "Numbers with zero magnitude are made dimensionless.")

;;; (make-pq 33.4 "meters/second")
;;; (make-pq 88 '(/ feet minute))
;;; (make-pq 5 'speed)
;;; #_5.0_KM/S

(defun make-pq (magnitude unit-expression &optional (scalar-dimension t))
  "Make a physical dimension quantity (PDQ).  Allow for the
   possiblity that magnitude is already a PDQ; in this case, check
   that the physical dimension is correct, and pass it on.  This can
   be used as a way of assigning default units to a number. Unitless dimensions (e.g., 'length) are
   interpeted as units in the :system-of-units.  If scalar-dimension is T and
   magnitude is not a scalar, the unit-expression applies to each
   element of the magnitude.  Otherwise it must be a grid or list of
   the shape as the magnitude."
  ;; Check that magnitude is a pdq
  (if (and (physical-quantity-p magnitude)
	   (check-dimension magnitude unit-expression))
      magnitude
      (let ((mag
	      ;; If the magnitude has pdq elements (i.e., array), take them out.
	      ;; Note, unlike above, no consistency check is made.
	      (if (grid:gridp magnitude)
		  (funcall (grid:elementwise #'pqval)
			   magnitude)
		  magnitude)))
	(if (and (grid:gridp mag) (not scalar-dimension))
	    (make-pq-grid
	     mag
	     (if (grid:gridp unit-expression)
		 unit-expression
		 (make-array
		  (grid:dimensions unit-expression)
		  :element-type t
		  :initial-contents unit-expression)))
	    (make-pq-ue mag unit-expression)))))

(defun make-pq-zero (unit-expression)
  (let ((*zero-is-dimensionless* nil)) (make-pq 0 unit-expression)))

(defun make-pq-ue (magnitude unit-expression)
  "Make the physical dimension quantity with a unit expression from a number or grid; if a grid, the same units apply to all elements."
  #+debug
  (check-type
   magnitude
   (and (not physical-quantity) (or number (satisfies grid:gridp))))
  (when (complexp magnitude)
    (cerror "Accept" "Magnitude ~a is complex." magnitude))
  (if (or (and *zero-is-dimensionless* (zerop magnitude))
	  (not unit-expression))
      magnitude
      (multiple-value-bind (dimel convfact)
	  (parse-units unit-expression)
	(if (equal dimel (dimension 'dimensionless))
	    magnitude
	    (if convfact
		(make-pq-object (antik:* convfact magnitude) dimel t)
		(error
		 "~a is not a unit of measure but a physical dimension ~
                      and cannot be interpreted as a unit of measure ~
                      without a system of units."
		 unit-expression))))))
