;; Number types used by C functions, and specification conversion
;; Liam Healy 2008-12-31 21:06:34EST types.lisp
;; Time-stamp: <2010-06-26 19:19:39EDT types.lisp>
;;
;; Copyright 2008, 2009 Liam M. Healy
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

(export '(*cstd-integer-types*
	  *cstd-cl-type-mapping* floating-point-association all-types
	  lookup-type cl-single cl-cffi cffi-cl number-class))

;;;;****************************************************************************
;;;; Type specification conversion
;;;;****************************************************************************

;; cffi-features:no-long-long doesn't work for me, but ought to be checked? 

;;; The different element type forms:
;;; C standard full, or "CFFI"  :unsigned-char
;;; C explicit                  :uint8     (not used here)
;;; CL                          '(unsigned-byte 8)
;;; Single                      'unsigned-byte-8 

;;; Functions to perform conversions
;;; CL -> single in function #'cl-single
;;; CL -> CFFI (Cstd) in function #'cl-cffi
;;; CFFI (Cstd) -> CL in function #'cffi-cl

;;; Sources of equivalence
;;; Cstd -> CL in alist *cstd-cl-type-mapping*

;;; :long-double may be pushed onto *features* if
;;; the implementation supports long doubles in CFFI.

;;;;****************************************************************************
;;;; Basic definition
;;;;****************************************************************************

;;; Preliminary definitions
(defparameter *cstd-integer-types*
  '(:char :unsigned-char
    :short :unsigned-short
    :int :unsigned-int
    :long :unsigned-long
    #+int64
    :long-long
    #+int64
    :unsigned-long-long)
  ;; http://common-lisp.net/project/cffi/manual/html_node/Built_002dIn-Types.html
  "List of integer types supported by CFFI, from the CFFI docs.")

(defparameter *fp-type-mapping*
  '((:float . single-float) (:double . double-float)
    ;; For those implementations that support a separate long-double
    ;; type assume this mapping:
    #+long-double (:long-double . long-float)
    (complex-float-c . (complex single-float))
    (complex-double-c . (complex double-float))
    ;; For those implementations that support a separate long-double
    ;; type assume this mapping:
    #+long-double
    (complex-long-double-c . (complex long-float)))
  ;; Ordered by: real shortest to longest, then complex shortest to longest.
  "List of floating point types supported by CFFI from the CFFI docs
   plus corresponding complex types.")

;;; Mapping alists used by conversion functions.

(defparameter *cstd-cl-type-mapping*
  (append
   *fp-type-mapping*
   (mapcar
    (lambda (type)
      (cons
       type
       (list
	(if (string-equal type "uns" :end1 3)
	    'unsigned-byte
	    'signed-byte)
	(* 8 (cffi:foreign-type-size type)))))
    *cstd-integer-types*))
  ;; Be careful when reverse associating, as there may be several C
  ;; types that map to a single CL type.
  "An alist of the C standard types as keywords, and the CL type
   The exception is complex types, which don't have a definition
   in the C standard; in that case, the C type is the foreign struct
   definition.")

(defmacro floating-point-association (splice-list)
  `(mapcar
   #'cons
   (mapcar #'first *fp-type-mapping*)
   (subseq ,splice-list 0 (length *fp-type-mapping*))))

;;;;****************************************************************************
;;;; Conversions
;;;;****************************************************************************

(defun all-types (alist &optional right-side)
  "A list of all types defined by symbol or definition."
  (mapcar (if right-side #'rest #'first) alist))

(defun lookup-type (symbol alist &optional reverse)
  "Lookup the symbol defined in the alist."
  (or 
   (if reverse
       (first (rassoc symbol alist :test #'equal))
       (rest (assoc symbol alist)))
   ;;(error "Did not find ~a in ~a" symbol (mapcar #'first alist))
   ))

;;; (cl-single '(unsigned-byte 8))
;;; UNSIGNED-BYTE-8
(defun cl-single (cl-type &optional (package *package*))
  "The element type name as a single symbol."
  (intern (if (atom cl-type)
	      (princ-to-string cl-type)
	      (format nil "~{~a~^-~}" cl-type))
	  package))

(defun cl-cffi (cl-type)
  "The CFFI element type from the CL type."
  (lookup-type (clean-type cl-type) *cstd-cl-type-mapping* t))

(defun cffi-cl (cffi-type)
  "The CL type from the CFFI element type."
  (unless (eq cffi-type :pointer)
    (lookup-type cffi-type *cstd-cl-type-mapping*)))

(defparameter *default-element-type* 'double-float)

(defun cffi-type (object)
  "The foreign element type of the object."
  (cl-cffi (element-type object)))

;;; For future use
(defparameter *cl-numeric-classes* '(ratio integer rational float real complex number))
(defun number-class (type)
  "Find the class corresponding to the numeric type."
  (find-if (lambda (class) (subtypep type class)) *cl-numeric-classes*))

