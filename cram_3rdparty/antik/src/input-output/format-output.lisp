;; Format numerical output
;; Liam Healy Wed Dec 11 2002 - 16:37
;; Time-stamp: <2014-02-07 10:25:03EST format-output.lisp>

;; Copyright 2011, 2013, 2014 Liam M. Healy
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

(export '(nf nf-string nf-option nf-readably set-nf-options
	  with-nf-options texstyle object-as-nf))

;;;;****************************************************************************
;;;; Parameters
;;;;****************************************************************************

(define-parameter nf :style
  :value 20
  :type (or null fixnum (member :tex :short :readable))
  :documentation
  "Style of format, plain (nil), LaTeX (:tex), or plain but (possibly) shortened (:short or a number).  If :short a short form will be used if available, if a fixnum is supplied, the short form will be used if available and the space available is less than the space taken by the long form.  If :readable, a form that is readable to CL is produced, if :reproducible a readble form is produced, and on reading object will be recreated exactly.")

;;;;****************************************************************************
;;;; Macros for parameter access
;;;;****************************************************************************

(defmacro nf-option (name)
  "Get/set the nf option named."
  `(parameter-value :nf ,name))

(defun nf-readably ()
  "The format produced will be readable to Lisp."
  (member (nf-option :style) '(:readable)))

(defmacro set-nf-options (&rest name-values)
  "Set the numerical formatting options."
  `(set-parameters :nf ,@name-values))

(defmacro with-nf-options ((&rest name-values) &body body)
  "Set the options for all nf calls within body to inherit."
  `(with-parameters (nf ,@(loop for (n v) on name-values by #'cddr collect (list n v)))
    ,@body))

(define-symbol-macro texstyle
  (and (eq (nf-option :style) :tex)))

;;;;****************************************************************************
;;;; General nf
;;;;****************************************************************************

(defmacro princn (object stream)
  `(if ,stream (princ ,object ,stream) ,object))

(defvar *nf-t-hook* nil
  "A list of a type and function, or nil. If an object passed to nf is of the type in the first element of the list, then the function in the second element is called on the object and the stream.")

(defgeneric nf (object &optional stream)
  (:documentation "Format output for numerical objects.  Default stream is *standard-output*.")
  (:method ((object t) &optional (stream *standard-output*))
    (if-defined (grid::gridp object)
		(nf-grid object)
		(princ object stream))))

(defun nf-string (object)
  "Format output for numerical objects to a new string."
  (with-output-to-string (stream)
    (nf object stream)))

(defmethod nf
    ((object null) &optional (stream *standard-output*))
  (princ "---" stream))

;;;;****************************************************************************
;;;; Numbers nf
;;;;****************************************************************************

(define-parameter nf :significant-figures :value 4
  :type (or null fixnum)
  :documentation
  "The number of significant figures formatted for numbers.
   If nil, formatting of numbers is done using :intpart-digits
   and, if a float, :fracpart-digits.")

(define-parameter nf :intpart-digits
  :value nil :type (or null fixnum)
  :documentation
  "The minimum space allowed for the whole-number part of numbers
   when :significant-figures is nil.  If this is larger than
   the actual number of digits, pad to the left with spaces.
   If this value is nil, allow enough space to accomodate the number.")

(define-parameter nf :print-sign
  :value nil :type t
  :documentation
  "Whether leading `+' is printed.")

(defmethod nf
    ((object integer) &optional (stream *standard-output*))
  (format stream "~vd"
	  (or (nf-option :significant-figures)
	      (nf-option :intpart-digits))
	  object))

(defmethod nf ((number ratio) &optional (stream *standard-output*))
  (if texstyle
      (format stream "~:[~;-~]\\frac{~d}{~d}"
	      (minusp number)
	      (abs (numerator number))
	      (denominator number))
    (call-next-method)))

;;; More in float.lisp

;;;;****************************************************************************
;;;; Readable format and re-creation of object as it is formatted 
;;;;****************************************************************************

(defun object-as-nf (object)
  "Define a new object exactly as the current nf options print an existing one.  This function can be used for example to define a number exactly as the rounded print format of another one."
  (read-from-string
   (with-nf-options (style :readable) (nf-string object))))

(defmacro cl-readable-nf (&body body)
  "For use in #'print-object methods that rely on nf methods for
   printing: they should print readably if cl:*print-escape* or
   cl:*print-readably* say to."
  `(with-nf-options
       (:style (if (readably) :readable (nf-option :style))
	       :full-precision (readably))
     ,@body))
