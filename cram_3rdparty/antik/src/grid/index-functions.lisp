;; Function to make grids based on functions of the indices
;; Liam Healy 2013-12-25 12:56:53EST index-functions.lisp
;; Time-stamp: <2013-12-28 11:09:32EST index-functions.lisp>

;; Copyright 2013 Liam M. Healy
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

(export '(make-grid-from-index make-grid-sequential-elements))

;;; Needed:
;;; Tests with destination
;;; Tests without destination
;;; Documentation/examples

;;; Examples
;;; (make-grid-from-index (lambda (i j) (+ j (* 10 i))) :dimensions '(4 4))
;;; (make-grid-from-index (lambda (i) (+ 4 (* 3 i))))
;;; (make-grid-sequential-elements :dimensions '(4 4) :grid-type 'foreign-array)
;;; (make-grid-sequential-elements :dimensions '(4 4) :element-type '(signed-byte 64))
;;; (make-grid-sequential-elements :dimensions '(4 4) :grid-type 'array)
;;; (make-grid-sequential-elements :dimensions '(4 4) :grid-type 'array :element-type 'fixnum)
;;; #2A((0 1 2 3) (10 11 12 13) (20 21 22 23) (30 31 32 33))

(defun make-grid-from-index
    (function
     &key
       destination
       destination-specification
       (grid-type *default-grid-type*)
       (dimensions *default-dimensions*)
       (element-type *default-element-type*))
  "Make or set a grid from the index or indices. The function gets the indices as arguments, and returns a value that will become the element at that index."
  (grid:map-grid
   :source function
   :destination-specification
   (or destination-specification (make-specification grid-type dimensions element-type))
   :destination destination))

(defun make-grid-sequential-elements
    (&rest args &key (offset 0) (step-col 1) (step-row 10) &allow-other-keys)
  "Create or set values in a grid with a value based on an affine transformation of the index or indices."
  (apply
   #'make-grid-from-index
   (lambda (i &optional j)
     (if j
	 (+ offset (* step-col j) (* step-row i))
	 (+ offset (* step-col i))))
   (alexandria:remove-from-plist args :offset :step-col :step-row)))

