;; GSL errors                                
;; Liam Healy Sat Mar  4 2006 - 18:33
;; Time-stamp: <2015-06-02 17:38:15EDT conditions.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2010, 2015 Liam M. Healy
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

(in-package :gsl)

;;;;****************************************************************************
;;;; Define non-error and error C return codes 
;;;;****************************************************************************

#.(append '(eval-when (:compile-toplevel :load-toplevel :execute))
	  (loop for i from (cffi:foreign-enum-value 'gsl-errorno :continue)
	     to (cffi:foreign-enum-value 'gsl-errorno :eof)
	     for name = (string (cffi:foreign-enum-keyword 'gsl-errorno i))
	     collect
	     `(defconstant ,(intern (format nil "+~:@(~a~)+" name) :gsll) ,i)))

;;;;****************************************************************************
;;;; GSL conditions
;;;;****************************************************************************

(export 'gsl-condition)
(define-condition gsl-condition (arithmetic-error warning)
  ((error-number :initarg :error-number :reader error-number)
   (error-text :initarg :error-text :reader error-text)   
   (explanation :initarg :explanation :reader explanation)
   (source-file :initform nil :initarg :source-file :reader source-file)
   (line-number :initform 0 :initarg :line-number :reader line-number))
  (:report
   (lambda (condition stream)
     (format stream "~a~@[; ~a~] ~@[in ~a at line ~d~]"
	     (error-text condition)
	     (explanation condition)
	     (source-file condition)
	     (line-number condition))))
  (:documentation
   "A condition that has been signalled by the GNU Scientific Library."))

#|
;;; This makes the printing of the condition object more informative,
;;; but also overrides the :report which I don't want to do.
(defmethod print-object ((object gsl-condition) stream)
  (print-unreadable-object (object stream :type t :identity t) 
    (format stream "~a (GSL condition)" (error-text object))))
|#

(defparameter *errorno-keyword* nil)

(defmacro define-gsl-condition (keyword number text &rest superclasses)
  `(progn
    (define-condition
	  ,keyword ,(or superclasses '(gsl-condition))
      ((error-number :initform ,number :reader error-number :allocation :class)
       (error-text :initform ,text :reader error-text :allocation :class))
      (:documentation
       ,(format nil
	       "The condition ~a, `~a,' signalled by the GNU Scientific Library."
	       keyword text)))
    (setf *errorno-keyword* (acons ,number ',keyword *errorno-keyword*))
    (export ',keyword)))

(define-condition unspecified-errno (gsl-condition)
  ((error-text :initform "Returned errno code not recognized"
	       :reader error-text :allocation :class))
  (:documentation
   "Errno value from GNU Scientific Library not recognized."))

(define-gsl-condition input-domain +edom+ "Input domain error")
(define-gsl-condition input-range +erange+ "Output range error")
(define-gsl-condition invalid-pointer +efault+ "Invalid pointer")
(define-gsl-condition invalid-argument +einval+ "Invalid argument")
(define-gsl-condition generic-failure-1 +efailed+ "Generic failure")
(define-gsl-condition generic-failure-2 +failure+ "Generic failure")
(define-gsl-condition factorization-failure +efactor+ "Factorization failed")
(define-gsl-condition sanity-check-failure
    +esanity+ "Sanity check failed - shouldn't happen")
(define-gsl-condition memory-allocation-failure +enomem+ "Malloc failed")
(define-gsl-condition bad-function-supplied
    +ebadfunc+ "Problem with user-supplied function")
(define-gsl-condition runaway-iteration
    +erunaway+ "Iterative process is out of control")
(define-gsl-condition exceeded-maximum-iterations
    +emaxiter+ "Exceeded max number of iterations")
(define-gsl-condition gsl-division-by-zero
    +ezerodiv+ "Tried to divide by zero" gsl-condition division-by-zero)
(define-gsl-condition invalid-tolerance
    +ebadtol+ "User specified an invalid tolerance")
(define-gsl-condition failure-to-reach-tolerance
    +etol+ "Failed to reach the specified tolerance")
(define-gsl-condition underflow +eundrflw+ "Underflow")
(define-gsl-condition overflow +eovrflw+ "Overflow")
(define-gsl-condition loss-of-accuracy +eloss+ "Loss of accuracy")
(define-gsl-condition roundoff-failure
    +eround+ "Failed because of roundoff error")
(define-gsl-condition nonconformant-dimensions
    +ebadlen+ "Matrix, vector lengths are not conformant")
(define-gsl-condition nonsquare-matrix +enotsqr+ "Matrix not square")
(define-gsl-condition singularity +esing+ "Apparent singularity detected")
(define-gsl-condition divergence +ediverge+ "Integral or series is divergent")
(define-gsl-condition unsupported-feature
    +eunsup+ "Requested feature is not supported by the hardware")
(define-gsl-condition unimplemented-feature
    +eunimpl+ "Requested feature not (yet) implemented")
(define-gsl-condition cache-limit-exceeded +ecache+ "Cache limit exceeded")
(define-gsl-condition table-limit-exceeded +etable+ "Table limit exceeded")
(define-gsl-condition no-progress +enoprog+
  "Iteration is not making progress towards solution")
(define-gsl-condition jacobian-not-improving
    +enoprogj+ "Jacobian evaluations are not improving the solution")
(define-gsl-condition failure-to-reach-tolerance-f
    +etolf+ "Cannot reach the specified tolerance in F")
(define-gsl-condition failure-to-reach-tolerance-x
    +etolx+ "Cannot reach the specified tolerance in X")
(define-gsl-condition failure-to-reach-tolerance-g
    +etolg+ "Cannot reach the specified tolerance in gradient")
;; not a subclass of gsl-condition
(define-gsl-condition gsl-eof +eof+ "End of file" end-of-file)
;;; It is possible to return +positive-infinity+
;;; by defining a handler for 'overflow.

(defun lookup-condition (number)
  (or (rest (assoc number *errorno-keyword*))
      'unspecified-errno))

(defun signal-gsl-error (number explanation &optional file line)
  "Signal an error from the GSL library."
  (unless (success-failure number)
    (let ((condition (lookup-condition number)))
      (if (eq condition 'unspecified-errno)
	  (error condition
	   :error-number number
	   :explanation explanation
	   :source-file file
	   :line-number line)
	  (error condition
	   :explanation explanation
	   :source-file file
	   :line-number line)))))

(defun signal-gsl-warning (number explanation &optional file line)
  "Signal a warning from the GSL library."
  (unless (success-failure number)
    (let ((condition (lookup-condition number)))
      (if (eq condition 'unspecified-errno)
	  (warn condition
		:error-number number
		:explanation explanation
		:source-file file
		:line-number line)
	  (warn condition
		:explanation explanation
		:source-file file
		:line-number line)))))

(cffi:defcallback gsl-error :void
    ((reason :string) (file :string) (line :int) (error-number :int))
  (signal-gsl-error error-number reason file line))

(defun establish-handler ()
  (cffi:foreign-funcall
   "gsl_set_error_handler"
   :pointer (cffi:callback gsl-error)))

(establish-handler)

;;; This insures that conditions will be signalled if GSLL is dumped
;;; in save-lisp-and-die.
#+sbcl (push 'establish-handler sb-ext:*init-hooks*)

;;; Convenience macro so user can specify a value to return for a particular error.
(export 'return-value-on-error)
(defmacro return-value-on-error (values error &body body)
  "Return the value(s) (a value or list of values) in case the specified GSL error is signalled in the body."
  `(restart-case
       (handler-bind
	   ((,error
	      #'(lambda (condition)
		  (declare (ignore condition))
		  (invoke-restart 'return ,@(alexandria:ensure-list values)))))
	 ,@body)
     (return (&rest v) (apply 'values v))))
