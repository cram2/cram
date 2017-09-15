;; Temporary file to figure out what to define to separate physical-quantities from grid definitions
;; Liam Healy 2013-03-16 11:46:04EDT separate-pq-grid.lisp
;; Time-stamp: <2013-03-16 12:11:33EDT separate-pq-grid.lisp>

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

(in-package :antik)

(defun format-units-scalar (value units &optional scalar-dimension (stream *standard-output*))
  "Format the pieces of the physical dimension quantity.  If formatting of
   unit alone is desired, value should be nil."
  (when units
    (if texstyle
	(if (and (listp units) (member (first units) '(/ cl:/)))
	    (format
	     stream "\\unitfrac~@[[~a]~]{~a}{~a}"
	     (when value (nf-string value))
	     (make-pq-string (second units) (nf-option style))
	     (make-pq-string (third units) (nf-option style)))
	    (format
	     stream
	     (if (eq units 'degree) "~@[~a~]~a" "\\unit~@[[~a]~]{~a}")
	     (when value (nf-string value))
	     (make-pq-string units (nf-option style)))))
    ;; Plain style
    (if scalar-dimension
	(format stream
		(if (nf-readably) "~@[#_~a_~]~a" "~@[~a~]~a")
		(when value (nf-string value))
		(make-pq-string units)))))

;; To be defined in #:pq-grid system
(defun format-units-grid (value units &optional scalar-dimension (stream *standard-output*))
  (if texstyle
      ;; Tex grid
      (progn
	(nf value stream)
	(format-units nil units scalar-dimension stream))
      ;; Plain, grid
      (loop for i from 0 below (grid:total-size value)
	    do
	       (format stream
		       (if (nf-readably) "~@[#_~a_~]~a" "~@[~a~]~a")
		       (when value (nf-string (grid:aref value i)))
		       (make-pq-string (grid:aref units i))))))

;;; Desired equivalent before grid is defined
(defun format-units (value units &optional scalar-dimension (stream *standard-output*))
  (format-units-scalar value units scalar-dimension stream))

;;; Desired equivalent after grid is defined
(defun format-units (value units &optional scalar-dimension (stream *standard-output*))
  (if (and value (grid:gridp value))
      (format-units-grid value units scalar-dimension stream)
      (format-units-scalar value units scalar-dimension stream)))

;;; Can be defined in Antik, without grid
(defun format-units (value units &optional scalar-dimension (stream *standard-output*))
  (if (and *gridp* object (funcall *gridp* object))
      (funcall (get 'format-units 'grid-function) value units scalar-dimension stream)
      (format-units-scalar value units scalar-dimension stream)))

;; To be defined in antik-base 
(defvar *gridp* nil)
(defmacro grid-switch (object scalar-body grid-function-arguments)
  ;;(if (and value (grid:gridp value))
  `(if (and *gridp* (funcall *gridp* object))
       ))

;; To be defined in grid
;; (setf (get 'format-units 'grid-function) #'format-units-grid)

(defmacro def-grid-function (name arglist &body body)
  `(progn
     (shadow ,name) ; this needs to be shadowed at compile time?
     (defun ,name arglist ,@body)
     (setf (get (alexandria:ensure-symbol name :antik) 'antik::grid-function)
	   ,name)))
