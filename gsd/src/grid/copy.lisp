;; Copy objects
;; Liam Healy 2009-12-21 10:16:04EST copy.lisp
;; Time-stamp: <2010-07-22 10:28:35EDT copy.lisp>
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

(export '(copy copy-to array-mismatch))

;;; Key arguments to #'copy:
;;; :specification :grid-type :dimensions :element-type
;;; :destination

;;; To copy to and from nested lists, use #'contents and the
;;; :initial-contents argument to make-grid, respectively.

;; 1. If destination object is not given, a specification must be given.
;; 2. A specification is :specification and zero or more of
;;    :grid-type :dimensions :element-type, or all of these three only.
;; 3. If :destination is a grid and none of the four
;;    :specification :grid-type :dimensions :element-type
;;    are, output is written into the destination.
;; 4. In any other case, a new grid is made according to the
;;    specification. 
;; 5. If none of the four are specified, a grid with the same
;;    specification is made and the contents copied over.

(defun copy-grid
    (object &key
     (specification (specification object))
     grid-type dimensions element-type
     destination)
  "Copy the grid to another grid."
  (map-grid :source object :destination destination
	    :destination-specification
	    (merge-specification
	     specification
	     grid-type dimensions element-type)))

(defgeneric copy
    (object &key specification grid-type dimensions element-type
	    destination)
  ;; This copies values into an existing object.
  (:documentation "Copy contents into existing or new object.")
  (:method ((object array) &rest args
	    &key specification grid-type dimensions element-type
	    destination)
    (declare (ignorable specification grid-type dimensions element-type
			destination))
    (apply 'copy-grid object args)))

(defun copy-to (object &optional (type *default-grid-type*))
  "Make a CL array from the object."
  (copy object :grid-type type))

(define-condition array-mismatch (error)
  ()
  (:report
   (lambda (condition stream)
     (declare (ignore condition))
     (format stream "Arrays must be of equal size for copying.")))
  (:documentation
   "An error indicating that the two arrays do not have 
   the same dimensions."))
