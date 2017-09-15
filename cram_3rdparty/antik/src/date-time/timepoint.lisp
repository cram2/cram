;; A point in time (specification with timescale)
;; Liam Healy Fri Oct 12 2001 - 11:53
;; Time-stamp: <2015-12-24 16:17:29EST timepoint.lisp>

;; Copyright 2011, 2015 Liam M. Healy
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

(export '(timepoint make-timepoint convert-time-scale
	  *timepoint-print-scale* *timepoint-print-scale-name*
	  clut-to-timepoint now today))

;;;****************************************************************************
;;; timepoint object
;;;****************************************************************************

(defparameter *timepoint-print-scale* :utc
  "Convert timepoints to this timescale for printing.
   If nil, use the internal time scale (UTC).")

(defparameter *timepoint-print-scale-name* nil
  "When T, print the scale (UTC, TAI, etc.) with a timepoint.")

(defclass timepoint (dtspec)
  ;; scale is :TAI or :UTC (or a timezone number?)
  ((scale :initarg :scale :reader scale))
  (:documentation "Specification of a point in time, including a scale."))

(defmethod print-object ((object timepoint) stream)
  (call-next-method
   (if *timepoint-print-scale*
       (convert-time-scale object *timepoint-print-scale*)
     object)
   stream)
  (if (or *timepoint-print-scale-name* (not (eq *timepoint-print-scale* :utc)))
      (princ (or *timepoint-print-scale* (symbol-name (scale object)))
	     stream)))

(defun make-timepoint
    (&rest args &key dtspec timeparse scale (day-only nil))
  "Make an object representing a point in time.  Either dtspec
   or timeparse should be specified, and scale should be specified."
  (declare (ignore dtspec timeparse day-only))
  (if (eq scale :utc)
      ;; If already utc, don't call convert: this allows
      ;; timepoints to be defined in this file.
      (apply #'make-timepoint-noconvert args)
    (convert-time-scale
     ;; If not UTC, convert: internally represent all times as UTC 
     (apply #'make-timepoint-noconvert args)
     :utc)))

(defun make-timepoint-noconvert
    (&key dtspec timeparse scale (day-only nil day-only-specified))
  "Make an object representing a point in time.  Either dtspec, or timeparse should be specified, and scale should be specified. Note that if dtspec is specified, the contents are changed in the original object."
  (assert scale (scale) "Scale must be specified.")
  (assert (or dtspec timeparse) (dtspec timeparse)
	  "One of dtspec or timeparse must be specified.")
  (let ((obj
	 (if dtspec
	     (change-class dtspec 'timepoint)
	     (make-dtspec
	      :type 'timepoint
	      :timeparse timeparse :day-only day-only))))
    (setf (slot-value obj 'scale) scale)
    (when day-only-specified
      (setf (slot-value obj 'day-only) day-only))
    obj))

(defmethod nf
    ((object timepoint) &optional (stream *standard-output*))
  (if (nf-option :timepoint-linear)
      ;; (with-nf-options (timepoint-linear :year) (nf now))
      ;; 2003.1234585963446
      (cond ((listp (nf-option :timepoint-linear))
	     (nf
	      (apply #'linear-timepoint object (nf-option :timepoint-linear))
	      stream))
	    ((string-equal (nf-option :timepoint-linear) :year)
	     (nf (linear-timepoint object) stream))
	    (t (error "nf-option timepoint-linear not recognized.")))
    (call-next-method)))

#|
(defmethod creation-form ((tp timepoint))
  (creation-form-readably
   tp
   `(make-timepoint
     :timeparse ',(timeparse tp)
     :day-only ,(day-only tp)
     :scale ,(scale tp))))
|#

(defmethod creation-form ((tp timepoint))
  `(make-timepoint
    :timeparse ',(timeparse tp)
    :day-only ,(day-only tp)
    :scale ,(scale tp)))

(def-make-load-form timepoint)

;;;****************************************************************************
;;; The current timepoint
;;;****************************************************************************

(defun clut-to-timepoint (clut &optional day-only)
  "Compute the timepoint from the CLUT."
  (make-timepoint
     :timeparse
     (clut-to-timeparse clut)
     :scale :utc
     :day-only day-only))

(define-symbol-macro now 
    ;; The current datime.
    (clut-to-timepoint (get-universal-time)))

(define-symbol-macro today
    ;; The current day.
    (clut-to-timepoint (get-universal-time) t))
