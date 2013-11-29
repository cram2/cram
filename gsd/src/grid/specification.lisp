;; The specification of a grid
;; Liam Healy 2009-11-21 14:48:34EST specification.lisp
;; Time-stamp: <2010-07-05 22:27:45EDT specification.lisp>
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

(export '(spec-scalar-p element-type))

;;; specification of an 8x3x2x4 grid:
;;; ((list 8) (list 3) (array 6 4) double-float)

(defun top-spec-type (specification)
  "The first, or top, type in the specification."
  (or (spec-scalar-p specification)
      (caar specification)))

(defun top-spec-dimensions (specification)
  (cdar specification))

(defun spec-scalar-p (specification)
  "Specification is for a scalar; if so, return that type."
  (when (not (rest specification))
    (first specification)))

(defun make-specification (type dimensions element-type)
  "Make a specification from the type, dimensions, and element-type."
  (assert (member type *grid-types*) (type)
	  "Specified type is not one of ~a" *grid-types*)
  ;; Do this assert for type=foreign-array only.
  #+(or)
  (assert (member element-type *array-element-types* :test 'equal)
	  (element-type)
	  "Specified element-type is not one of ~a" *array-element-types*)
  `((,type ,@(alexandria:ensure-list dimensions)) ,element-type))

(defun specification-dimensions (specification)
  "The grid dimensions from its specification."
  (mapcan 'rest (butlast (copy-tree specification))))

(defun spec-element-type (spec)
  "The element type of the specification."
  (alexandria:lastcar spec))

(defun merge-specification
    (specification &optional type dimensions element-type)
  "Make a new specification based on the existing one, replacing values
   as given."
  (make-specification
   (or type (and specification (top-spec-type specification)))
   (or dimensions (and specification (top-spec-dimensions specification)))
   (or element-type (and specification (spec-element-type specification)))))
