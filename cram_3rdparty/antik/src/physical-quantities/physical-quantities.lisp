;; Objects that represent physical measurements.                             
;; Liam Healy Wed Mar  6 2002 - 09:04
;; Time-stamp: <2015-11-28 22:24:53EST physical-quantities.lisp>

;; Copyright 2011, 2012, 2013, 2014, 2015 Liam M. Healy
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

(export '(pqval check-dimension with-pq))

;;;;****************************************************************************
;;;; Parsing unit names
;;;;****************************************************************************

;;; (parse-units "m-kg^2/s^2")
;;; (parse-units 'm-kg^2/s^2)
;;; (parse-units '(/ (* M KG KG) (* S S)))
;;; All return
;;; (1 -2 2 0 0 0 0 0 0)
;;; 1.0
(defun parse-units (unit-expression)
  "Find the dimel representation of the dimensions from a string, symbol, or unit sexp."
  (parse-unit-sexp (find-unit-expr (parse-pq-string unit-expression))))

;;; (parse-pq-string "m-kg^2/s^2")
;;; (/ (* M KG KG) (* S S))
(defun parse-pq-string (string)
  "Parse a string or symbol specifying units into lists."
  (if (listp string)
      string
      (flet ((symbolize (str)
	       (when (plusp (length str))
		 ;; Empty strings become nil, e.g. "/m" is (/ nil meter)
		 (antik-symbol str))))
	(flet ((lexprod (s)
		 (let ((trms
			(mapcan
			 (lambda (x)
			   (let ((prod (split-sequence:split-sequence #\^ x)))
			     (if (single prod)
				 (list (symbolize (first prod)))
				 (make-list
				  (parse-integer (second prod))
				  :initial-element (symbolize (first prod))))))
			 (split-sequence:split-sequence-if
			  (lambda (x) (member x '(#\- #\*)))
			  s))))
		   (if (single trms) (first trms) (cons '* trms)))))
	  (let ((str (if (symbolp string) (symbol-name string) string)))
	    (destructuring-bind (num &optional den)
		(split-sequence:split-sequence #\/ str)
	      (if den
		  (list '/ (lexprod num) (lexprod den))
		  ;; / with no numerator is reciprocal
		  (if (eq (cl:aref str 0) #\/)
		      (list '/ (lexprod num))
		      (lexprod num)))))))))

(defun check-dimension-or-type (object type)
  "Return T if the object is of the type specified, and, if a physical-quantity, is of the physical-dimension specified in type."
  (or (if (and (typep object 'physical-quantity)
	       (ignore-errors (antik::parse-unit-sexp type)))
	  (check-dimension object type nil))
      (typep object type)))

(setf *parameter-check-function* 'check-dimension-or-type)

(defun check-dimension
    (obj units
     &optional (errorp t) (zeros-have-any-dimension *zero-is-dimensionless*))
  "T if one of these cases hold:
    - obj is a pdq and units have the same dimension as obj,
    - obj is zero and zeros-have-any-dimension is T,
    - obj and units represent a dimensionless quantity,
    - obj and units are grids of the same length, and for 
      each pair of corresponding elements, one of the above is true."
  (if (physical-quantity-p obj)
      (if (and obj (grid:gridp obj) (not (scalar-dimension obj)))
	  (loop for i from 0 below (grid:total-size obj)
		always (equal-dimension (grid:aref* obj i) (grid:aref* obj i)))
	  (equal-dimension obj (parse-units units) errorp zeros-have-any-dimension t))
      (eq units 'dimensionless)))

(defun equal-dimension
    (x y
     &optional
       (errorp t)
       (zeros-have-any-dimension *zero-is-dimensionless*)
       match-dimensionless)
  "If the pdq are equal physical dimension, return that dimel.
   If one quantity is zero and zeros-have-any-dimension, return the
   dimel of the other and disregard whether they're the same physical
   dimension.  If y represents a dimensionless quantity,
   and match-dimensionless is true, return the dimel of x.
   Return two values: the dimel and whether it is scalar dimension."
  (flet ((err-return ()
	   (when errorp (error "The quantities ~a and ~a are not both physical ~
               quantities with the same physical dimension." x y))))
    (cond ((and zeros-have-any-dimension (zerop x))
	   (values (dimel-or-not y) (scalar-dimension y)))
	  ((and zeros-have-any-dimension (zerop y))
	   (values (dimel-or-not x) (scalar-dimension x)))
	  ((and (eql (scalar-dimension x) (scalar-dimension y))
		(equalp (dimel-or-not x) (dimel-or-not y)))
	   ;; Covers the case of both having non-scalar units or both having scalar units
	   (values (dimel-or-not x) (scalar-dimension x)))
	  ((and match-dimensionless (equal (dimel-or-not y) (dimel-or-not nil)))
	   (dimel-or-not x))
	  ;; If only one has non-scalar units, break out the individual dimensions.
	  ((and (grid:gridp x) (grid:gridp y)
		(or (not (scalar-dimension y)) (not (scalar-dimension x))))
	   ;; All same units, possibly one has scalar dimension
	   (multiple-value-bind (they-are value)
	       (all-same	       ; repurposing 2nd return value 
		(concatenate
		 'list
		 (if (scalar-dimension x) (list (pq-dimension x)) (pq-dimension x))
		 (if (scalar-dimension y) (list (pq-dimension y)) (pq-dimension y)))
		:test #'cl:equal)
	     (if they-are
		 (values they-are value)
		 (err-return))))
	  (t (err-return)))))

(defun dimel-or-not (object)
  "Find the dimel for this object (a physical dimension quantity or a dimel), or
   return the dimensionless dimel."
  (cond ((physical-quantity-p object)
	 (values (pq-dimension object) (scalar-dimension object)))
	((dimelp object) object)
	(t (dimension 'dimensionless))))

(defun equal-dimensions
    (list &optional (zero-is-dimensionless *zero-is-dimensionless*))
  "If all elements of the list have the same physical dimension, return that dimel."
  (let* ((flist (alexandria:flatten list))
	 (short-list (if zero-is-dimensionless (remove-if #'zerop flist) flist)))
    (all-same
     short-list
     :test (alexandria:rcurry #'equal-dimension nil)
     :post-function #'dimel-or-not)))

;;;;****************************************************************************
;;;; Extracting values and printing
;;;;****************************************************************************

(defvar *time-interval-format-function* #'fare-utils:nilf)

(defmethod nf
    ((pq physical-quantity) &optional (stream *standard-output*))
  (if (and (grid:gridp pq)		
	   (not (and (typep pq 'physical-quantity) (scalar-dimension pq))))
      ;; If a grid that is either not a pdq or is a pdq with non-scalar
      ;; dimensions, call the T method of nf, so that it will call
      ;; nf-grid.
      (call-next-method)
      (or (funcall *time-interval-format-function* pq stream)
	  (multiple-value-bind (val units)
	      (pqval pq)
	    (if (nf-option :system-of-units)
		(format-units val units (scalar-dimension pq) stream)
		(nf val stream))))))	; format the number only

(defmethod print-object ((pq physical-quantity) stream)
  "Print the physical-quantity using nf."
  #+debug
  (print-unreadable-object (pq stream)
    (format stream "Physical dimension quantity [debug] mag ~a"
	    (pq-magnitude pq)))
  #-debug
  (let ((dim (pq-dimension pq)))
    (if (or (null (nf-option :system-of-units))
	    (and dim (arrayp dim) (null (row-major-aref dim 0))))
	;; units not set yet
	(print-unreadable-object (pq stream)
	  (format stream "Physical dimension quantity mag ~a, dimensions ~a"
		  (pq-magnitude pq)
		  (when dim
		    (with-system-of-units
			(basis-physical-dimensions)
		      (make-unit-sexp dim t)))))
	(cl-readable-nf (nf pq stream)))))

(defgeneric pqval (pq)
  (:documentation
   "Get the numerical value and the units of the physical dimension quantity.  Returns the magnitude, the units, and whether units are the same for each element of a sequence or grid.")
  (:method (pq)
    (values pq nil nil)))

(defmethod pqval ((pq physical-quantity))
  (if (scalar-dimension pq)
      (let ((units (make-unit-sexp (pq-dimension pq) t)))
	(values
	 (antik:/ (pq-magnitude pq) (nth-value 1 (parse-unit-sexp units)))
	 units
	 t))
      (values
       (grid:map-grid
	:source pq
	:destination-specification (grid:specification (pq-magnitude pq))
	:element-function 'pqval)
       (grid:map-grid
	:source (pq-dimension pq)
	:destination-specification (grid:specification (pq-dimension pq))
	:element-function (alexandria:rcurry 'make-unit-sexp t)))))

(defparameter *pqval-allsame-sequence-collapse* t
  "If a sequence passed to pqval has all the same units,
   collapse to one unit specification.")

(defmethod pqval ((pq sequence))
  (let ((mat
	 (transpose-list
	  (map 'list
	       (lambda (x)
		 (multiple-value-list
		  (pqval x)))
	       pq))))
    (multiple-value-bind (allsame-val allsame-same)
	(all-same (second mat) :test #'equal) ; all same units
      (let* ((allsame-nz
	      ;; all non-zero quantities have the same units
	      (when (and *pqval-allsame-sequence-collapse*
			 (not allsame-same)
			 *zero-is-dimensionless*)
		;; Check again if zeros are gone
		(let ((removed-zeros (remove-if 'zerop pq)))
		  (when (< (length removed-zeros) (length pq))
		    (multiple-value-bind (ignore units asnz)
			(pqval removed-zeros)
		      (declare (ignore ignore))
		      (when asnz units))))))
	     (as (and *pqval-allsame-sequence-collapse*
		      (or allsame-same allsame-nz))))
	(values
	 (if (vectorp pq) (coerce (first mat) 'vector) (first mat))
	 ;; If all units are the same, return them, else return a list of units.
	 (if as
	     (if allsame-same allsame-val allsame-nz)
	     (second mat))
	 as)))))

(defun pqval-seq (pq)
  (let ((*pqval-allsame-sequence-collapse* nil))
    (pqval pq)))

;;;;****************************************************************************
;;;; Interface
;;;;****************************************************************************

(defun format-units (value units &optional scalar-dimension (stream *standard-output*))
  "Format the pieces of the physical dimension quantity.  If formatting of
   unit alone is desired, value should be nil."
  (when units
    (if texstyle
	(if (and value (grid:gridp value))
	    (progn
	      (nf value stream)
	      (format-units nil units scalar-dimension stream))
	    (if (and (listp units) (member (first units) '(/ cl:/)))
		(format
		 stream "\\unitfrac~@[[~a]~]{~a}{~a}"
		 (when value (nf-string value))
		 (make-pq-string (second units) (nf-option :style))
		 (make-pq-string (third units) (nf-option :style)))
		(if (and (eq (nf-option :degrees) :dms) (eq units 'degree))
		    (format-dms value stream)
		    (format
		     stream
		     (if (eq units 'degree) "~@[~a~]~a" "\\unit~@[[~a]~]{~a}")
		     (when value (nf-string value))
		     (make-pq-string units (nf-option :style))))))
	;; Plain style
	(if scalar-dimension
		(if (and (eq (nf-option :degrees) :dms) (eq units 'degree))
		(format-dms value stream)
		(format stream
			(if (nf-readably) "~@[#_~a_~]~a" "~@[~a~]~a")
			(when value (nf-string value))
			(make-pq-string units)))
	    ;; iterate over the grid
	    (loop for i from 0 below (grid:total-size value)
		  do
		     (format stream
			     (if (nf-readably) "~@[#_~a_~]~a" "~@[~a~]~a")
			     (when value (nf-string (grid:aref value i)))
			     (make-pq-string (grid:aref units i))))))))

(defmacro with-pq (physical-quantities &body body)
  "Ensure that the named physical dimension quantities are of the right dimensions, or convert them from plain numbers using the current system of units as specified by (nf-option :system-of-units)."
  ;; should I have the option to return plain numbers
  ;; in some system of units as well?
  `(let ,(mapcar
	  ;; check that value is non-nil before make-pq
	  (lambda (vu)
	    (destructuring-bind (v u) vu `(,v (and ,v (make-pq ,v ',u)))))
	  physical-quantities)
     ,@body))

(defun read-pq-from-string (string units)
  "Read the next number and interpret as a pdq, if units is non-nil.
   The special 'pointer gives the starting location to read
   and is updated after the read." 
  (declare (special pointer))
  (multiple-value-bind (mag ptr)
      (read-from-string
       string nil nil :start (if (boundp 'pointer) pointer 0))
    (setf pointer ptr)
    (if units (make-pq mag units) mag)))

(defun scalar-units-p (units)
  (ignore-errors (parse-units units)))

(defun read-physical-dimension-quantity (stream subchar arg)
  "Read the physical dimension quantity from the stream."
  (declare (ignore subchar arg))
  (let ((*zero-is-dimensionless* nil)
	numerical-value units)
    ;; Tell the reader to read the things
    (setf numerical-value (stream-to-string stream :terminator #\_)
	  units (read stream t nil t))
    ;; Make the PDQ if reading is not suppressed
    (unless *read-suppress*
      (make-pq
       (read-from-string numerical-value)
       units
       (scalar-units-p units)))))

;;; Use #_ for physical dimension quantities, e.g. #_90_degree
(set-dispatch-macro-character
 #\# #\_ 'read-physical-dimension-quantity
 (named-readtables:find-readtable :antik))

(defmethod creation-form ((pq physical-quantity))
  (multiple-value-bind (mag units)
      (pqval pq)
    ;; Units should probably have a creation-form on it but I don't
    ;; think this will do anything useful in practice, makes for a
    ;; more cluttered form.
    `(make-pq ,(creation-form mag) ',units)))

(def-make-load-form physical-quantity)
