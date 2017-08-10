;; Angles by component parts.
;; Liam Healy Thu May 30 2002 - 11:46
;; Time-stamp: <2014-10-06 22:29:48EDT angle-component.lisp>

;; Copyright 2011 Liam M. Healy
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

(export '(angle asine acosine atangent arise arun sine consine
	  rise run normalize-angle))

;;; Angles may be defined by "components", that is, trigonometric functions of
;;; the angle, halfplanes, etc.  Components may be defined without the whole
;;; angle being known, with the rest of the information to be supplied later. 
;;; At the point enough information is supplied to define the angle,
;;; the value may be extracted with pq-magnitude, which will happen
;;; automatically if it is printed.

;;; Examples:
;;; 1) A normal definition with a magnitude
;;; (setf normal-def (make-instance 'angle :magnitude 1.234))

;;; 2) Define "rise" and "run" (fixed multiples of sine and cosine);
;;;   note that a symbol label is required to confirm the same multiplier:
;;; (setf (rise true-anomaly 'eccentricity) 0.02)
;;; (setf (run true-anomaly 'eccentricity) 0.04)
;;; true-anomaly is now #_0.46365_RADIANS.

;;; 3) Define an angle with the cosine and the vertical halfplane:
;;; (setf (cosine ang) 0.34)
;;; (setf (vertical-halfplane ang) -1)
;;; Is #_-1.2239_RADIANS.

;;;;****************************************************************************
;;;; Angle object
;;;;****************************************************************************

#|
;;; EXPERIMENTAL
(defclass angle (physical-quantity)
  ((magnitude :initform nil)
   (dimension :initform (dimension 'angle))
   (sine :initform nil :initarg :sine :reader sine
	 :writer sinecomp :type (or nil number))
   (cosine :initform nil :initarg :cosine :reader cosine
	   :writer cosinecomp :type (or nil number))
   (tangent :initform nil :initarg :tangent
	    :accessor tangent :type (or nil list number))
   (vertical-halfplane :initform nil :initarg :vertical-halfplane
		       :accessor vertical-halfplane
		       :documentation "Sign of sine"
		       :type (or nil (integer -1 1)))
   (horizontal-halfplane :initform nil
			 :initarg :horizontal-halfplane
			 :accessor horizontal-halfplane
			 :documentation "Sign of cosine"
			 :type (or nil (integer -1 1)))
   (normalized :initform nil :initarg :normalized :accessor normalized
	       :documentation "Fractional part of whole revolution."
	       ;; may be -pi to pi, 0 to 2pi, etc.
	       :type t)
   (whole-revolutions :initform 0 :initarg :whole-revolutions
		      :accessor whole-revolutions
		      :documentation
		      "Number of complete revolutions, which when added to normalized,
                       gives the angle."
		      :type integer))
  (:documentation "Angles with component parts."))

;;;;****************************************************************************
;;;; Angle consistency, definition, component extraction, printing
;;;;****************************************************************************

(defmacro with-angle (angle &body body)
  `(with-slots
       (magnitude dimension sine cosine tangent
	vertical-halfplane horizontal-halfplane
	normalized whole-revolutions)
       ,angle
     ,@body))

(defun check-angle-consistency (angle)
  "Check that the components of the angle are consistent if defined,
   and return the angle."
  (check-type angle angle)
  (with-angle angle
    (when (and sine cosine)
      (assert (antik:zerop (-nz (cl:+ (cl:expt sine 2) (cl:expt cosine 2)) 1.0))
	  (sine cosine)
	"Sine and cosine do not both belong to the same angle."))
    (when (and sine cosine tangent (physical-quantity-p tangent))
      (assert (antik:zerop (-nz (cl:/ sine cosine) tangent))
	  (sine cosine tangent)
	"Tangent is not the ratio of sine and cosine."))
    (when (and sine vertical-halfplane)
      (assert (eql (signum sine) vertical-halfplane)
	  (sine vertical-halfplane)
	"Sine and vertical halfplane do not agree."))
    (when (and cosine horizontal-halfplane)
      (assert (eql (signum cosine) horizontal-halfplane)
	  (cosine horizontal-halfplane)
	"Cosine and horizontal halfplane do not agree.")))
  angle)

(defun make-angle (&rest args)
  (check-angle-consistency (apply #'make-instance 'angle args)))

(defmethod print-object ((angle angle) stream)
  (if (magnitude-or-nil angle)
      (call-next-method)
    (print-unreadable-object (angle stream :identity t)
      (with-angle angle
	(format
	 stream
	 ;;"Partially known angle:~#[ none~; ~S~; ~S and ~S~:;~@{~#[~; and~] ~S~^,~}~]"
	 "Partially known angle: ~:{~(~s~)=~d~:^, ~}"
	 (remove nil
		 `((sine ,sine) (cosine ,cosine) (tangent ,tangent)
				(vertical-halfplane ,vertical-halfplane)
				(horizontal-halfplane ,horizontal-halfplane)
				(normalized ,normalized)
				(whole-revolutions ,whole-revolutions))
		 :key #'second))))))

;;;;****************************************************************************
;;;; Trigonometric functions
;;;;****************************************************************************

#+(or) ; uses pythagorean-complement
(defmethod cos ((angle angle))
  (with-angle angle
    (or cosine
	(setf cosine
	  (cond
	   ((magnitude-or-nil angle) (antik:cos (pq-magnitude angle)))
	   ((and sine horizontal-halfplane)
	    (antik:* horizontal-halfplane (pythagorean-complement sine)))
	   ((and tangent vertical-halfplane)
	    (antik:* vertical-halfplane
		(/ (sqrt (1+ (expt (antik:tan angle) 2))))))))
	(error "Insufficient information to compute cosine."))))

#+(or) ; uses pythagorean-complement
(defmethod sin ((angle angle))
  (with-angle angle
    (or sine
	(setf sine
	  (cond
	   ((magnitude-or-nil angle) (antik:sin (pq-magnitude angle)))
	   ((and cosine vertical-halfplane)
	    (antik:* vertical-halfplane (pythagorean-complement cosine)))
	   ((and tangent vertical-halfplane)
	    (antik:* horizontal-halfplane
		(/ (antik:tan angle) (sqrt (1+ (expt (antik:tan angle) 2))))))))
	(error "Insufficient information to compute sine."))))

(defmethod tan ((angle angle))
  (with-angle angle
    (if (and tangent (physical-quantity-p tangent))
	tangent
      (setf tangent
	(cond
	 ((magnitude-or-nil angle) (antik:tan (pq-magnitude angle)))
	 (tangent
	  (cl:/ (first tangent) (second tangent)))
	 ((or sine cosine)
	  (cl:/ (antik:sin angle) (antik:cos angle)))
	 (t (error "Insufficient information to compute tangent.")))))))

;;;;****************************************************************************
;;;; Defining an angle by setting components
;;;;****************************************************************************

(defun acosine (cosine &optional angle)
  "Start defining an angle by specifying its cosine.
   Returns the angle object."
  (check-angle-consistency
   (if angle
       (progn (cosinecomp angle cosine) angle)
     (make-angle :cosine cosine))))

(defun asine (sine &optional angle)
  "Start defining an angle by specifying its sine.
   Returns the angle object."
  (check-angle-consistency
   (if angle
       (progn (sinecomp angle sine) angle)
     (make-angle :sine sine))))

(defun atangent (tangent &optional angle)
  "Define an angle by specifying the tangent.  Returns the angle object."
  (if angle
      (progn (setf (tangent angle) tangent)
	     angle)
    (make-angle :tangent tangent)))

(defun arise (rise &optional label angle)
  "Make or modify an angle by components, label is a symbol
   naming the multiplier of sine.  Returns the angle object."
  (if (and angle (tangent angle)
	   ;; If the run is already defined, 
	   (listp (tangent angle))
	   ;; check label
	   (eql (first (tangent angle)) label))
      ;; and replace
      (progn
	(setf (first (tangent angle)) rise)
	angle)
    ;; otherwise, make a new angle object
    (atangent (list rise label) nil)))

(defun arun (run &optional label angle)
  "Make or modify an angle by components, label is a symbol
   naming the multiplier of cosine.  Returns the angle object." 
  (if (and angle (tangent angle)
	   ;; If the run is already defined, 
	   (listp (tangent angle))
	   ;; check label
	   (eql (second (tangent angle)) label))
      ;; and replace
      (progn
	(setf (second (tangent angle)) run)
	angle)
    ;; otherwise, make a new angle object
    (atangent (list label run) nil)))

;;;;****************************************************************************
;;;; Setf expansions to set an angle's components
;;;;****************************************************************************

(defmacro place-unbound-p (place)
  "Is the place unbound?"
  `(typep (nth-value 1 (ignore-errors ,place)) 'cell-error))

(defun angle-component-writer-form (form angle sv newval setter getter)
  "The writer-form in the setf-expansion of the angle."
  `(progn
     ;; If ,getter is not bound, set to nil so that it may be
     ;; reset to the angle object.
     ;; boundp does not work on non-symbols (general places)
     ;; boundp will not detect lexical bindings.
     (when (place-unbound-p ,getter)
       (eval (list 'let (list (list (first ',newval) nil)) ',setter)))
     (setf ,angle ,form)
     ,sv))

(defmacro set-angle-component (angle setform env)
  "Define an expansion to setf an angle component.  The setform should
   have an argument ,sv which is the value to set."
  `(multiple-value-bind (dummies vals newval setter getter)
       (get-setf-expansion angle ,env)
     (alexandria:with-gensyms (sv)
       (values
	 dummies
	 vals
	 (list sv)
	 (angle-component-writer-form
	  ,setform
	  ,angle sv newval setter getter)
	 nil))))

(define-setf-expander rise (angle label &environment env)
  "Set the rise of an angle, e.g.
    (setf (rise true-anomaly 'eccentricity) 0.02).
   The label is a symbol naming the multiplier of sine.
   It is used to determine that the same multiplier was used for the run."
  (set-angle-component angle `(arise ,sv ,label ,angle) env))

(define-setf-expander run (angle label &environment env)
  "Set the run of an angle, e.g.
    (setf (run true-anomaly 'eccentricity) 0.02).
   The label is a symbol naming the multiplier of cosine.
   It is used to determine that the same multiplier was used for the rise."
  (set-angle-component angle `(arun ,sv ,label ,angle) env))

;;; These setf expanders are better than the :accessor because it is
;;; not necessary to define an angle object first.  2004-06-17 These
;;; are now the only ones; changed slot definition to be :reader
;;; because sbcl objected.
(define-setf-expander cosine (angle &environment env)
  "Set the cosine of an angle."
  (set-angle-component angle `(acosine ,sv ,angle) env))

(define-setf-expander sine (angle &environment env)
  "Set the sine of an angle."
  (set-angle-component angle `(asine ,sv ,angle) env))

;;;;****************************************************************************
;;;; Angle value
;;;;****************************************************************************

;;; Conditional relations for finding the angle
;;; angle = whole-revolutions + normalized
;;; normalized = (atan tangent)
;;; tangent = (list sine cosine)
;;; normalized = (* horizontal-halfplane (acos cosine))
;;; normalized =
;;;  (let ((asin (asin sine)))
;;;     (if (minusp vertical-halfplane) (- pi asin) (if (minusp asin) (+ 2pi asin) asin)))

(defparameter *half-rev* (make-pq 1/2 'revolution))

(defmethod pq-magnitude :before ((angle angle))
  ;; I'm undecided on whether to use antik:atan etc. here vs. atan;
  ;; I'd like to work strictly in radians, but it might be necessary
  ;; to process generic numbers like bigfloats.
  (with-angle angle
    ;; Try to solve for the angle when component information is present
    (when (null magnitude)
      (unless normalized
	(setf normalized
	      (cond ((or tangent (and sine cosine))
		     (unless tangent (setf tangent (list sine cosine)))
		     (if (and (listp tangent) (not (some #'symbolp tangent)))
			 ;; two numerical values for rise/run in tangent slot; compute
			 (apply #'antik:atan tangent)
			 (cond (vertical-halfplane
				(antik:+ (if (minusp vertical-halfplane)
					     *half-rev* 0)
					 (antik:atan tangent)))
			       (horizontal-halfplane
				(let ((atan (antik:atan tangent)))
				  (antik:+
				   (if (cl:minusp (cl:* atan horizontal-halfplane))
				       *half-rev* 0)
				   atan)))
			       (t nil))))
		    ((and cosine vertical-halfplane)
		     (antik:* vertical-halfplane (antik:acos cosine)))
		    ((and sine horizontal-halfplane)
		     (let ((asin (antik:asin sine)))
		       (if (minusp horizontal-halfplane)
			   (antik:- *half-rev* asin) ; result between pi/2, 3pi/2
			   asin)))	; result between -pi/2, pi/2
		    (t (error "Insufficient information to find angle.")))))
      (setf
       magnitude
       (pqval (antik:+ normalized (make-pq whole-revolutions 'revolution)))))))

(defun magnitude-or-nil (angle)
  (ignore-errors (pq-magnitude angle)))
|#

;;;;****************************************************************************
;;;; Angle normalization
;;;;****************************************************************************

(defgeneric normalize-angle (angle &optional positive)
  (:documentation "Normalizes the angle to (in radians) -pi to pi
   or 0 to 2pi (positive = t).
   Two values are returned: the normalized angle, and the number
   of full revolutions that should be added to the normalized angle
   to recreate the original angle.")
  (:method ((angle number) &optional positive)
	   (multiple-value-bind (revs norm)
	       (funcall (if positive #'antik:floor #'antik:round) 
			angle 2pi)
	     (values norm revs))))

#|
(defmethod normalize-angle ((angle physical-quantity) &optional positive)
  (if (check-dimension angle 'angle nil)
      (let ((*zero-is-dimensionless* nil))
	(with-pq ((angle angle))
	  (unless (typep angle 'angle)
	    ;; Object may be a pdq with angle dimension, but not yet
	    ;; an angle object, so change it.
	    (change-class angle 'angle))
	  (with-angle angle
	    (multiple-value-bind (norm revs)
		(normalize-angle (pq-magnitude angle) positive)
	      (setf whole-revolutions revs
		    normalized (make-pq norm 'radian))
	      (values norm revs)))))
    angle))
|#

(defmethod normalize-angle ((angle physical-quantity) &optional positive)
  (if (check-dimension angle 'angle nil)
      (let ((*zero-is-dimensionless* nil))
	(with-pq ((angle angle))
	  (unless (typep angle 'angle)
	    ;; Object may be a pdq with angle dimension, but not yet
	    ;; an angle object, so change it.
	    (change-class angle 'angle))
	  (multiple-value-bind (norm revs)
	      (normalize-angle (pq-magnitude angle) positive)
	    (values (make-pq norm 'radian) revs))))
      angle))

(defmethod normalize-angle ((seq sequence) &optional positive)
  (map (type-of seq)
    (lambda (angle)
      (if (check-dimension angle 'angle nil)
	  (normalize-angle angle positive)
	angle))
    seq))

#|
;;; Formerely norm-denorm-angle
(defun apply-function-to-normalized-angle (angle function &optional positive)
  "Normalize the angle, apply the function, and denormalize it back to the
   original cycle count.  This is useful for e.g. Kepler's equation,
   where the normalized angle is needed to for the algorithm to
   converge properly, but the full cycles are needed for f & g functions."
  (normalize-angle angle positive)
  (antik:+ (funcall function (normalized angle))
	   (make-pq (whole-revolutions angle) 'revolution)))
|#
