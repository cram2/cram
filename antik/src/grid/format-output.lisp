;; Format numerical output for grids and sequences
;; Liam Healy Wed Dec 11 2002 - 16:37
;; Time-stamp: <2015-04-26 22:45:46EDT format-output.lisp>

;; Copyright 2011, 2013, 2014, 2015 Liam M. Healy
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

;;;;****************************************************************************
;;;; Grids
;;;;****************************************************************************

(defmethod nf ((object array) &optional (stream *standard-output*))
      (nf-grid object stream))

;;;;****************************************************************************
;;;; Sequences nf
;;;;****************************************************************************

(define-parameter nf :vector-format
  :value :horizontal :type (member :horizontal :vertical :coordinate-unit-vectors)
  :documentation
  "Rank-1 grids (vectors) are formatted as rows, columns or as linear
  combination of coordinate unit vectors.")

(define-parameter nf :components
  :value '("I" "J" "K") :type list
  :documentation
  "Names of vector components to use :vector-format is :coordinate-unit-vectors.")

(define-parameter nf :tex-element-separator
  :value (format nil " \\\\~%") :type string
  :documentation
  "What to put between vertically separated elements for LaTeX.")

(define-parameter nf :tex-decimal-align
  :value t :type boolean
  :documentation
  "Align columns of numbers on decimal point in LaTeX.")

(define-parameter nf :vertical-element-separator
  :value #\Newline :type character
  :documentation
  "What to put between vertically separated elements for plain style.")

(define-parameter nf :horizontal-element-separator
  :value #\space :type character
  :documentation
  "What to put between horizontally separated elements for plain style.")

(defun vector-component-names (i)
  "The name of the ith vector component."
  (let ((source (nf-option :components)))
    (elt source (mod i (length source)))))

(defmethod nf ((object sequence) &optional (stream *standard-output*))
  (when (grid:gridp object)
    (nf-grid object stream)))

   #+(or)
   (format stream "~a~{~a~^ ~})"
	   (if (vectorp object) "#(" "'(")
	   (map 'list (lambda (o) (nf-string o)) object))

(defun nf-grid (object &optional (stream *standard-output*))
  "Format the grid."
  (grid::print-grid-readably
   (when (nf-readably) object)
   (lambda (s)
     (ecase (grid:rank object)
       (1 (with-nf-options (:vector-format (if (nf-readably) :horizontal (nf-option :vector-format)))
	    (if (member (nf-option :vector-format) '(:horizontal :vertical))
		(nf-matrix object s)
		(nf-vector object s))))
       (2 (nf-matrix object s))))
   stream)
  object)

;;; Set the hook in the T method for #'nf so that the nf-grid is called if the argument is a grid.
(setf *nf-t-hook* (list 'grid:grid #'nf-grid))

(defun nf-vector (object &optional (stream *standard-output*))
  "Format the one dimensional grid."
  ;; Note: cannot handle lists yet.
  ;; This is called from nf-grid, called only when :vector-format is :coordinate-unit-vectors.
  (dotimes (i (if (numberp *print-length*)
		  (min *print-length* (grid:dim0 object))
		  (grid:dim0 object)))
    ;; iterate formatting each element
    (with-nf-options
	;; If formatting as a linear combination with element labels,
	;; force a + sign except for the first element
	;; to make a mathematical expression.
	(print-sign
	 (and (eq (nf-option :vector-format) :coordinate-unit-vectors)
	      (not (zerop i))))
      ;; format the element itself
      (nf (grid:aref object i) stream)
      ;; append either the element label or the separator
      (princ
       (if (nf-readably)
	   #\space
	   (if texstyle
	       (case (nf-option :vector-format)
		 (:coordinate-unit-vectors (format nil "\\unitvec{~a}" (vector-component-names i)))
		 (:vertical "\\\\")
		 (:horizontal " "))
	       (case (nf-option :vector-format)
		 (:coordinate-unit-vectors (vector-component-names i))
		 (:vertical #\newline)
		 (:horizontal #\space))))
       stream)))
  (when (and (numberp *print-length*) (cl:> (grid:dim0 object) *print-length*))
    (if texstyle
	(princ "\\dots" stream)
	(princ "..." stream)))
  object)

(defun parenstyle (outer)
  "Parentheses should be used in this case."
  (and outer
       (or (member (nf-option :style) '(:readable))
	   (numberp (nf-option :style)))))

(defun nf-matrix (object &optional (stream *standard-output*))
  "Format the two dimensional grid, or the one dimensional grid horizontally or vertically."
  (flet ((element (i j)
	   (if (grid:dim1 object)
	       (grid:aref object i j)
	       (case (nf-option :vector-format)
		 (:vertical (grid:aref object i))
		 (:horizontal (grid:aref object j))))))
    (let* ((dim1n (or (grid:dim1 object)
		      (if (eq (nf-option :vector-format) :vertical)
			  1
			  (grid:dim0 object))))
	   (dim0n (if (or (grid:dim1 object) (eq (nf-option :vector-format) :vertical))
		      (grid:dim0 object)
		      1))
	   (dim0 (if (numberp *print-length*) (min dim0n *print-length*) dim0n))
	   (dim1 (if (numberp *print-length*) (min dim1n *print-length*) dim1n)))
      (when texstyle
	(format stream "\\left[\\begin{array}{~a}~&"
		(if (nf-option :tex-decimal-align)
		    (reduce (alexandria:curry #'concatenate 'string)
			    (make-list dim1 :initial-element "r@{.}l"))
		    (make-string dim1 :initial-element #\r))))
      (when (parenstyle t) (princ #\( stream)) ; open paren for whole array
      (dotimes (i dim0)
	(when (parenstyle (grid:dim1 object)) (princ #\( stream)) ; open paren for a row
	(dotimes (j dim1)
	  (if (and texstyle (nf-option :tex-decimal-align))
	      (princ
	       (substitute #\& #\. (nf-string (element i j)))
	       stream)
	      (nf (element i j) stream))
	  (when (< j (1- dim1))
	    (if texstyle
		(princ " & " stream)
		(princ (nf-option :horizontal-element-separator) stream))))
	(when (parenstyle (grid:dim1 object)) (princ #\) stream)) ; close paren for a row
	(when (< i (1- dim0))
	  (if texstyle
	      (princ (nf-option :tex-element-separator) stream)
	      (princ (nf-option :vertical-element-separator) stream))))
      (when (and (numberp *print-length*) (or (> dim0n *print-length*) (> dim1n *print-length*)))
	(if texstyle
	    (princ "\\dots" stream)
	    (princ "..." stream)))
      ;; close out TeX array
      (when texstyle (format stream "~&\\end{array}\\right]"))
      (when (parenstyle t) (princ #\) stream)))) ; close paren for whole array
  nil)

