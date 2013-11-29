;; Iterate
;; Norman Werner 2009-05-26 22:23:40EDT iterate.lisp
;; Time-stamp: <2010-07-17 23:13:35EDT iterate.lisp>
;;
;; Copyright 2009 Norman Werner, Liam M. Healy
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

(in-package :iter)

#|
As an example try:

(defparameter *array-3-4-double-float* (test-grid-double-float 'array '(3 4)))

(iter:iter (iter:for e :matrix-element *array-3-4-double-float*)
	   (princ e) (princ " "))

(iter:iter (iter:for (row column) :matrix-element-index *array-3-4-double-float*)
	   (princ (grid:gref *array-3-4-double-float* row column)) (princ " "))
|#

(defclause-sequence matrix-row matrix-row-index
  :access-fn
  '(lambda (grid index)
    (assert (and (grid:gridp grid) (eql (grid:grid-rank grid) 2))
     (grid))
    (grid:row grid index))
  :size-fn
  '(lambda (grid)
    (assert (and (grid:gridp grid) (eql (grid:grid-rank grid) 2))
     (grid))
    (first (grid:dimensions grid)))
  :element-type t :sequence-type t
  :element-doc-string "(copied) rows of a matrix"
  :index-doc-string "index of the rows in a matrix")

(defclause-sequence matrix-column matrix-column-index
  :access-fn
  '(lambda (grid index)
    (assert (and (grid:gridp grid) (eql (grid:grid-rank grid) 2))
     (grid))
    (grid:column grid index))
  :size-fn
  '(lambda (grid)
    (assert (and (grid:gridp grid) (eql (grid:grid-rank grid) 2))
     (grid))
    (second (grid:dimensions grid)))
  :element-type t :sequence-type t
  :element-doc-string "(copied) columns of a matrix"
  :index-doc-string "index of the columns in a matrix")

(defclause-sequence vector-element vector-element-index
  :access-fn
  '(lambda (vector index)
    (assert (and (grid:gridp vector) (eql (grid:grid-rank vector) 1))
     (vector))
    (grid:gref vector index))
  :size-fn
  '(lambda (vector)
    (assert (and (grid:gridp vector) (eql (grid:grid-rank vector) 1))
     (vector))
    (first (grid:dimensions vector)))
  :element-type t :sequence-type t
  :element-doc-string "(copied) elements of a vector"
  :index-doc-string "index of elements in a vector")

(defmacro-driver (FOR element matrix-element matrix)
  "Iterates over all (copied) elements in matrix"
  (alexandria:with-unique-names (row-index col-index row-size col-size m)
      (when generate
	(error "Not yet implemented a generate clause for matrix-element."))
      `(progn
	 (with ,m = ,matrix)
	 (with ,row-index = 0)
	 (with ,col-index = 0)
	 (with ,row-size = (first (grid:dimensions ,m)))
	 (with ,col-size = (second (grid:dimensions ,m)))
	 (for ,element
	      :next
	      (if (>= ,row-index ,row-size)
		  (terminate)
		  (if (>= ,col-index ,col-size)
		      (progn
			(setf ,col-index 0)
			(incf ,row-index)
			(if (>= ,row-index ,row-size)
			    (terminate)
			    (grid:gref ,m ,row-index ,col-index)))
		      (prog1
			  (grid:gref ,m ,row-index ,col-index)
			(incf ,col-index))))))))

(defmacro-driver (FOR indexes matrix-element-index matrix)
  "Iterates over the indexes in matrix. indexes is a list like (row-index-name
   column-index-name) "
  (alexandria:with-unique-names (row-index col-index row-size col-size m)
    (when generate
      (error "Not yet implemented a generate clause for matrix-element-index."))
    `(progn
       (with ,m = ,matrix)
       (with ,row-index = 0)
       (with ,col-index = 0)
       (with ,row-size = (first (grid:dimensions ,m)))
       (with ,col-size = (second (grid:dimensions ,m)))
       (for ,indexes next (if (>= ,row-index ,row-size)
			      (terminate)
			      (if (>= ,col-index ,col-size)
				  (progn
				    (setf ,col-index 0)
				    (incf ,row-index)
				    (if (>= ,row-index ,row-size)
					(terminate)
					(list ,row-index ,col-index)))
				  (prog1
				      (list ,row-index ,col-index)
				    (incf ,col-index))))))))
