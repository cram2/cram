;; AFFI transformers
;; Tamas Papp and Liam Healy
;; Time-stamp: <2010-08-26 23:20:54EDT transformers.lisp>
;;
;; Copyright 2008, 2009 Tamas Papp and Liam M. Healy
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

(in-package :affi)

(export '(permute transpose subrange drop diagonal extrude stride))

;;;;****************************************************************************
;;;; Application of transformers
;;;;****************************************************************************

(defun apply-transformers (affi &rest transformers)
  "Create a new affi with the transformers applied in succession."
  (loop for (function arguments) on transformers by #'cddr
     with current-affi = affi
     do (when function
	  (setf current-affi (apply function current-affi arguments)))
     finally (return current-affi)))

;;;;****************************************************************************
;;;; Specific AFFI transformers
;;;;****************************************************************************

(defun permute (affi permutation)
  "Permute the subscripts of an affine index using the given list."
  (with-slots (const coeff domain) affi
    (let* ((rank (rank affi))
	   (flags (make-array rank :element-type 'bit :initial-element 0))
	   (new-coeff (make-fixnum-vector rank))
	   (new-domain (make-fixnum-vector rank)))
      (assert (= (length permutation) rank))
      (loop
	 for p :in permutation
	 for i :from 0
	 do
	 ;; check if permutation is valid
	 (cond
	   ((or (< p 0) (<= rank p)) (error "index ~a is out of range" p))
	   ((plusp (aref flags p)) (error "index ~a occurs at least twice" p))
	   (t (setf (aref flags p) 1)))
	 ;; new indexes
	 (setf (aref new-coeff i) (aref coeff p)
	       (aref new-domain i) (aref domain p)))
      ;; create new affi
      (make-instance 'affi :const const :coeff new-coeff :domain new-domain))))

(defun transpose (affi &optional (indices '(0 1)))
  "Transpose any pair of indices."
  (permute
   affi
   (loop for i below (rank affi)
      collect (if (member i indices)
		  (first (remove i indices))
		  i))))

(defun subrange-int (affi range)
  "Constrain an affine map to a subrange, which is given as a list of
ranges.  For details, see parse-range.  Return the new affine index."
  (with-slots (const coeff domain) affi
    (let* ((rank (rank affi))
	   (new-const const)
	   (new-coeff (make-fixnum-vector rank))
	   (new-domain (make-fixnum-vector rank)))
      (unless (= (rank affi) (length range))
	(error "range specification does not match rank of affine index"))
      (loop
	 for i :from 0
	 for d across domain
	 for c across coeff
	 for r :in range
	 do
	 (multiple-value-bind (start length) (parse-range r d)
	   (setf (aref new-domain i) (abs length)
		 (aref new-coeff i) (if (minusp length) (- c) c))
	   (incf new-const (* start c))))
      (make-instance 'affi :const new-const 
		     :coeff new-coeff :domain new-domain))))

(defun subrange (affi dims &optional dim-positions start)
  "Create an affi selecting a subrange from the given affi corresponding
   to the dimensions dims in the positions dim-positions."
  (unless dim-positions
    (setf dim-positions (loop for i below (length dims) collect i)))
  (unless (= (length dim-positions) (length dims))
    (error
     "Size of dims (~d) should be the same as the size of dim-positions (~d)"
     (length dims) (length dim-positions)))
  (subrange-int
   affi
   (let ((start-inds
	  (or start (make-list (length (get-domain affi)) :initial-element 0))))
     (mapcar
      'list
      start-inds
      (let ((end-inds (copy-list start-inds)))
	(loop for i below (length dim-positions)
	   do
	   (incf (elt end-inds (nth i dim-positions))
		 (1- (elt dims i))))
	end-inds)))))

(defun drop (affi &optional (which t))
  "Drop the dimensions from the domain which have size 1, provided
that their index is in `which', or `which' is t.  Return new affine
index.  If `which' is nil, no dimension is dropped."
  (unless which
    ;; we are not allowed to drop any
    (return-from drop affi))
  (unless (eq which t)
    (assert (listp which)))
  (with-slots (const coeff domain) affi
    (multiple-value-bind (new-coeff new-domain)
	(loop
	   ;; mark dimensions to drop
	   for i :from 0
	   for c across coeff
	   for d across domain
	   unless (and (= d 1) (or (eq which t) (member i which)))
	   collect d :into new-domain
	   unless (and (= d 1) (or (eq which t) (member i which)))
	   collect c :into new-coeff
	   finally (return (values new-coeff new-domain)))
      (make-instance 'affi :const const
		     :coeff (copy-into-fixnum-vector new-coeff)
		     :domain (copy-into-fixnum-vector new-domain)))))

(defun remove-position (sequence position &optional (result-type 'list))
  "Remove the element at position."
  (concatenate
   result-type
   (subseq sequence 0 position)
   (when (< (1+ position) (length sequence))
     (subseq sequence (1+ position)))))

(defun diagonal (affi &optional (offset 0) (indices '(0 1)))
  "Select a subarray where the two indices are equal or 
   differ by the offset, e.g. the diagonal affi for the matrix.
   The offset specifies sub- (offset<0) or super- (offset>0)
   diagonals."
  ;; See notes Sat Nov 21 2009
  (let* ((dims (get-domain affi))
	 (nrows (elt dims (first indices)))
	 (ncols (elt dims (second indices)))
	 (min-index (apply 'min indices))
	 (max-index (apply 'max indices))
	 (coeff (get-coeff affi)))
    (make-instance
     'affi
     :const (if (minusp offset) (* (abs offset) ncols) offset)
     :coeff
     (let ((seq (remove-position coeff max-index '(vector fixnum))))
       (setf (elt seq min-index)
	     (+ (elt coeff (first indices))
		(elt coeff (second indices))))
       seq)
     :domain
     (let ((seq (remove-position dims max-index '(vector fixnum))))
       (setf (elt seq min-index)
	     ;; Place the combined index range at the location of the first
	     (if (minusp offset)
		 (min (+ nrows offset) ncols)
		 (min nrows (- ncols offset))))
       seq))))

(defun extrude (affi index new-dimension)
  "Create a new AFFI that makes the grid look as if it has an
  additional dimension, with each element replicated along that
  dimension."
  (make-affi-int
   (insert-at new-dimension (coerce (get-domain affi) 'list) index)
   (insert-at 0 (coerce (get-coeff affi) 'list) index)))

(defun stride (affi stride)
  "Create a new AFFI that makes the grid look as if is
   been reduced by taking only every stride-th element."
  (make-affi-int
   (copy-into-fixnum-vector
    (list
     (ceiling
      (grid::total-size-from-dimension (get-domain affi))
      stride)))
   (list stride)
   (get-const affi)))
