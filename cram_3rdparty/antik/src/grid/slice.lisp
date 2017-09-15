;; Select a range or specific values of indices
;; Liam Healy 2009-12-05 16:16:46EST slice.lisp
;; Time-stamp: <2010-05-23 10:18:07EDT slice.lisp>
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

(export '(slice))

(defun slice-no-select (grid index-selection &key destination drop)
  (let* ((range
	  (mapcar
	   (lambda (ind)
	     (cond ((integerp ind) (list ind ind))
		   ((member ind '(:all :rev)) ind)
		   ((and (listp ind) (eql (first ind) :range))
		    (rest ind))
		   ((and (listp ind) (eql (first ind) :select))
		    ;; not correct, just take the first index
		    (list (second ind) (second ind)))))
	   index-selection))
	 (affi (affi::subrange-int (affi grid) range)))
    (map-grid :source grid
	      :source-affi affi
	      :destination destination
	      :destination-affi
	       (affi:drop (affi:make-affi (coerce (affi:get-domain affi) 'list))
			  drop))))

(defun replace-stride (selection)
  "Replace an index specified as :range with a stride with the
   explicit indices needed."
  (if (and (listp selection)
	   (eql (first selection) :range)
	   (= 4 (length selection)))
      (cons :select
	    (loop for i
	       from (second selection)
	       upto (third selection)
	       by (fourth selection)
	       collect i))
      selection))

(defun map-to-select (index-selection)
  (let* ((pos
	  (position-if
	   (lambda (list) (and (listp list) (eql (first list) :select)))
	   index-selection)))
    (values
     (if pos
	 (let ((selection (rest (nth pos index-selection))))
	   (loop for i below (length selection)
	      for ind in selection
	      collect
	      (set-position index-selection pos ind)))
	 (list index-selection))
     pos)))

(defun slice (grid index-selection &key destination drop)
  "Select slice(s) from a grid.  The index-selection is a list with
   length equal to the rank of grid.   Each element should be one of:
     an index, indicating the index to be selected,
     :all, indicating the entire range if indices are to be selected,
     :rev, indicating the entire range if indices are to be selected
         in reverse order,
     (:range start end stride), indicating a range of indices to be
         selected; if stride is omitted, it is presumed to be 1,
     (:select value ...), indicating explicit values to be selected."
  ;; The stride in :range is implemented by changing it into a select.
  ;; It should in fact map into an affi though.
  (multiple-value-bind (sels axis)
      (map-to-select (mapcar 'replace-stride index-selection))
    (if axis
	(loop for sel in sels
	   for accum =
	   (slice grid sel :drop nil)
	   then
	   (concatenate-grids
	    accum
	    (slice grid sel :drop nil)
	    :axis axis)
	   finally (return (drop accum :drop drop)))
	(slice-no-select
	 grid (first sels) :destination destination :drop drop))))
