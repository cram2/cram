;; Conditions and handlers for arithmetic.
;; Liam Healy Tue May 17 2005 - 16:29
;; Time-stamp: <2015-12-17 15:42:30EST conditions.lisp>

;; Copyright 2011, 2014, 2015 Liam M. Healy
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

(export '(coerce-undefined coerce-nil making-complex-number accept-coerce
	  handling-complex-number arithmetic-errors-return-nan))

;;;;****************************************************************************
;;;; Conditions
;;;;****************************************************************************

(define-condition coerce-undefined (arithmetic-error)
  ((object :initarg :object :reader coerce-object)
   (to :initarg :to :reader coerce-to))
  (:report (lambda (condition stream)
	     (format stream "Cannot coerce ~a to ~a."
		     (coerce-object condition)
		     (coerce-to condition))))
  (:documentation
   "Signalled when making a generalized number in which a higher type
    (like taylor-number) is a component of a lower type (like physical-quantity)."))

(define-condition coerce-nil (arithmetic-error)
  ((object :initarg :object :reader coerce-object))
  (:report (lambda (condition stream)
	     (format stream "Cannot coerce ~a to NIL." (coerce-object condition))))
  (:documentation "Signalled when coercing something to nil."))

(define-condition null-argument (warning)
  ()
  (:report (lambda (condition stream)
	     (declare (ignore condition))
	     (format stream "NIL passed as argument to arithmetic function.")))
  (:documentation "Warned when nil passed as argument to arithmetic function."))

(define-condition making-complex-number (arithmetic-error)
  ((operation :initarg :operation :reader mcn-operation)
   (number :initarg :number :reader mcn-number))
  (:report
   (lambda (condition stream)
     (format stream "The operation ~a on ~a will create a complex number."
	     (mcn-operation condition)
	     (mcn-number condition))))
  (:documentation "Signalled when a mathematical calculation would result in a complex number."))

;;;;****************************************************************************
;;;; Handlers
;;;;****************************************************************************

(defmacro handling-complex-number (restart &body body)
  "A handler to make the complex number."
  `(handler-bind
       ((making-complex-number
	 #'(lambda (c) (declare (ignorable c))
		   (invoke-restart ',restart))))
     ,@body))

(defun nan-warning ()
  (warn "Generating NAN")
  :nan)

(defmacro arithmetic-errors-return-nan (&body body)
  "Return a NaN for a variety of arithmetic errors."
  `(restart-case
       (handler-bind ((division-by-zero	; division by 0 produces :nan
		       #'(lambda (condition)
			   (declare (ignore condition))
			   (nan-warning)
			   (invoke-restart 'return :nan)))
		      ;; arithmetic with :nan produces :nan
		      (type-error
		       #'(lambda (condition)
			   (when (eq (type-error-datum condition) :nan)
			     (invoke-restart 'return :nan)))))
	 ,@body)
     (return (&optional v) v)))

(defmacro accept-coerce (&body body)
  "A handler to accept coerced object."
  `(handler-bind
       ((error #'(lambda (c)
		   (let ((r (find-restart 'continue c)))
		     (when r (invoke-restart r))))))
     ,@body))

#+global-handler
(setf *debugger-hook*
      (lambda (condition old-hook)
	(if (need-to-handle-conndition-specilaly condition)
	    blah
	    (funcall old-hook condition old-hook))))
