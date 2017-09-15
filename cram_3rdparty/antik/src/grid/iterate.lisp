;; Iterate
;; Norman Werner 2009-05-26 22:23:40EDT iterate.lisp
;; Time-stamp: <2015-11-19 22:42:29EST iterate.lisp>
;;
;; Copyright 2009, 2013, 2014, 2015 Norman Werner, Liam M. Healy
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

(defparameter *array-3-4-double-float* (make-grid-sequential-elements :dimensions '(3 4) :grid-type 'array :element-type 'double-float))

(iter:iter (iter:for e :matrix-element *array-3-4-double-float*)
	   (princ e) (princ " "))

(iter:iter (iter:for (row column) :matrix-element-index *array-3-4-double-float*)
	   (princ (grid:aref *array-3-4-double-float* row column)) (princ " "))
|#

(defclause-sequence matrix-row matrix-row-index
  :access-fn
  '(lambda (grid index)
    (assert (and (grid:gridp grid) (eql (grid:rank grid) 2))
     (grid))
    (grid:row grid index))
  :size-fn
  '(lambda (grid)
    (assert (and (grid:gridp grid) (eql (grid:rank grid) 2))
     (grid))
    (first (grid:dimensions grid)))
  :element-type t :sequence-type t
  :element-doc-string "(copied) rows of a matrix"
  :index-doc-string "index of the rows in a matrix")

(defclause-sequence matrix-column matrix-column-index
  :access-fn
  '(lambda (grid index)
    (assert (and (grid:gridp grid) (eql (grid:rank grid) 2))
     (grid))
    (grid:column grid index))
  :size-fn
  '(lambda (grid)
    (assert (and (grid:gridp grid) (eql (grid:rank grid) 2))
     (grid))
    (second (grid:dimensions grid)))
  :element-type t :sequence-type t
  :element-doc-string "(copied) columns of a matrix"
  :index-doc-string "index of the columns in a matrix")

;;; Define the defclause-sequence on vectors for iter:for.  To be consistent with matrix elements, this shouldn't be defined, but it is used in macroexpansion to define the antik:for clauses, and there's little harm in leaving it in.
(defclause-sequence vector-element vector-element-index
  :access-fn
  '(lambda (vector index)
    (assert (and (grid:gridp vector) (eql (grid:rank vector) 1))
     (vector))
    (grid:aref vector index))
  :size-fn
  '(lambda (vector)
    (assert (and (grid:gridp vector) (eql (grid:rank vector) 1))
     (vector))
    (first (grid:dimensions vector)))
  :element-type t :sequence-type t
  :element-doc-string "(copied) elements of a vector"
  :index-doc-string "index of elements in a vector")

;;; Define the defclause-sequence on vectors for antik:for
(PROGN
 (DEFCLAUSE-DRIVER (antik:FOR VAR VECTOR-ELEMENT SEQ &SEQUENCE)
   "(copied) elements of a vector"
   (RETURN-SEQUENCE-CODE :ELEMENT-VAR VAR :SEQUENCE SEQ :ACCESS-FN
                         '(LAMBDA (VECTOR INDEX)
                            (ASSERT
                             (AND (GRID:GRIDP VECTOR)
                                  (EQL (GRID:RANK VECTOR) 1))
                             (VECTOR))
                            (GRID:AREF VECTOR INDEX))
                         :SIZE-FN
                         '(LAMBDA (VECTOR)
                            (ASSERT
                             (AND (GRID:GRIDP VECTOR)
                                  (EQL (GRID:RANK VECTOR) 1))
                             (VECTOR))
                            (FIRST (GRID:DIMENSIONS VECTOR)))
                         :ELEMENT-TYPE T :SEQUENCE-TYPE T))
 (DEFCLAUSE-DRIVER (antik:FOR VAR VECTOR-ELEMENT-INDEX SEQ &SEQUENCE)
   "index of elements in a vector"
   (COND
    (WITH-INDEX
     (CLAUSE-ERROR "WITH-INDEX should not be specified for this clause"))
    (T (SETQ WITH-INDEX VAR)
     (RETURN-SEQUENCE-CODE :SEQUENCE SEQ :SIZE-FN
                           '(LAMBDA (VECTOR)
                              (ASSERT
                               (AND (GRID:GRIDP VECTOR)
                                    (EQL (GRID:RANK VECTOR) 1))
                               (VECTOR))
                              (FIRST (GRID:DIMENSIONS VECTOR)))
                           :SEQUENCE-TYPE T)))))

(defmacro-driver (antik:FOR element matrix-element matrix)
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
       (iter:for ,element
	    :next
	    (if (>= ,row-index ,row-size)
		(terminate)
		(if (>= ,col-index ,col-size)
		    (progn
		      (setf ,col-index 0)
		      (incf ,row-index)
		      (if (>= ,row-index ,row-size)
			  (terminate)
			  (prog1
			      (grid:aref ,m ,row-index ,col-index)
			    (incf ,col-index))))
		    (prog1
			(grid:aref ,m ,row-index ,col-index)
		      (incf ,col-index))))))))

(defmacro-driver (antik:FOR indexes matrix-element-index matrix)
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
       (iter:for ,indexes next (if (>= ,row-index ,row-size)
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
