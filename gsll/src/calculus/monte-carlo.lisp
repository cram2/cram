;; Monte Carlo Integration
;; Liam Healy Sat Feb  3 2007 - 17:42
;; Time-stamp: <2016-06-14 23:07:35EDT monte-carlo.lisp>
;;
;; Copyright 2007, 2008, 2009, 2011, 2012, 2016 Liam M. Healy
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

;;; /usr/include/gsl/gsl_monte.h
;;; /usr/include/gsl/gsl_monte_plain.h
;;; /usr/include/gsl/gsl_monte_miser.h
;;; /usr/include/gsl/gsl_monte_vegas.h

;;;;****************************************************************************
;;;; PLAIN Monte Carlo
;;;;****************************************************************************

(defmobject monte-carlo-plain
    "gsl_monte_plain"
  ((dim :sizet))
  "plain Monte Carlo integration"
  :documentation			; FDL
  "Make and initialize a workspace for Monte Carlo integration in dimension dim."
  :initialize-suffix "init"
  :initialize-args nil)

(defparameter *monte-carlo-default-samples-per-dimension* 150000)

(defmfun monte-carlo-integrate-plain
    (function lower-limits upper-limits 
	      &optional
	      (number-of-samples
	       (* *monte-carlo-default-samples-per-dimension*
		  (dim0 lower-limits)))
	      (generator (make-random-number-generator +mt19937+ 0))
	      (state (make-monte-carlo-plain (dim0 lower-limits)))
	      (scalars t))
  "gsl_monte_plain_integrate"
  ((callback :pointer)
   ((grid:foreign-pointer lower-limits) :pointer) ((grid:foreign-pointer upper-limits) :pointer)
   ((dim0 lower-limits) :sizet) (number-of-samples :sizet)
   ((mpointer generator) :pointer)
   ((mpointer state) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :inputs (lower-limits upper-limits)
  :callbacks
  (callback (:struct fnstruct-dimension) (dimension)
	    (function :double (:input :double :cvector dim0) :slug))
  :callback-dynamic (((dim0 lower-limits)) (function scalars))
  :documentation			; FDL
  "Uses the plain Monte Carlo algorithm to integrate the
   function f over the hypercubic region defined by the
   lower and upper limits in the arrays 'lower-limits and
   'upper-limits, each a gsl-vector of length dim.
   The integration uses a fixed number
   of function calls number-of-samples, and obtains random sampling points using
   the random number generator 'generator. A previously allocated workspace
   'state must be supplied.  The result of the integration is returned
   with an estimated absolute error.")

;;;;****************************************************************************
;;;; MISER
;;;;****************************************************************************

;;; The MISER algorithm of Press and Farrar is based on recursive
;;; stratified sampling.  This technique aims to reduce the overall
;;; integration error by concentrating integration points in the
;;; regions of highest variance.

(defmobject monte-carlo-miser
    "gsl_monte_miser"
  ((dim :sizet))
  "miser Monte Carlo integration"
  :documentation			; FDL
  "Make and initialize a workspace for Monte Carlo integration in
   dimension dim.  The workspace is used to maintain
   the state of the integration."
  :initialize-suffix "init"
  :initialize-args nil)

(defmfun get-mcm-parameters (object)
  "gsl_monte_miser_params_get"
  (((mpointer object) :pointer) (params (:pointer (:struct miser-params))))
  :c-return :void
  :return (params)
  :documentation "Get all parameters, as a foreign struct, for the MISER method."
  :export nil
  :index parameter
  :gsl-version (1 13))

(defmethod parameter ((object monte-carlo-miser) parameter)
  (cffi:foreign-slot-value (get-mcm-parameters object) 'miser-params parameter))

(defmfun set-mcm-parameters (object params)
  "gsl_monte_miser_params_set"
  (((mpointer object) :pointer) (params (:pointer (:struct miser-params))))
  :documentation "Set the parameter for the MISER method."
  :c-return :void
  :return (params)
  :export nil
  :index (setf parameter)
  :gsl-version (1 13))

(defmethod (setf parameter) (value (object monte-carlo-miser) parameter)
  (let ((current-params (get-mcm-parameters object)))
    (setf (cffi:foreign-slot-value current-params '(:struct miser-params) parameter)
	  value)
    (set-mcm-parameters object current-params)))

;;; As of GSL v1.13, the API above is used to get/set the MISER parameters, so the following is removed.
#+obsolete-gsl (export 'miser-parameter)
#+obsolete-gsl
(defmacro miser-parameter (workspace parameter)
  ;; FDL
  "Get or set with setf the parameter value for the MISER Monte Carlo
   integration method."
  ;; (miser-parameter ws min-calls)
  ;; (setf (miser-parameter ws min-calls) 300)
 `(cffi:foreign-slot-value ,workspace 'miser-state ',parameter))

(defmfun monte-carlo-integrate-miser
    (function lower-limits upper-limits 
	      &optional
	      (number-of-samples
	       (* *monte-carlo-default-samples-per-dimension*
		  (dim0 lower-limits)))
	      (generator (make-random-number-generator +mt19937+ 0))
	      (state (make-monte-carlo-miser (dim0 lower-limits)))
	      (scalars t))
  "gsl_monte_miser_integrate"
  ((callback :pointer)
   ((grid:foreign-pointer lower-limits) :pointer) ((grid:foreign-pointer upper-limits) :pointer)
   ((dim0 lower-limits) :sizet) (number-of-samples :sizet)
   ((mpointer generator) :pointer)
   ((mpointer state) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :inputs (lower-limits upper-limits)
  :callbacks
  (callback (:struct fnstruct-dimension) (dimension)
	    (function :double (:input :double :cvector dim0) :slug))
  :callback-dynamic (((dim0 lower-limits)) (function scalars))
  :documentation			; FDL
  "Uses the miser Monte Carlo algorithm to integrate the
   function f over the hypercubic region defined by the
   lower and upper limits in the arrays 'lower-limits and
   'upper-limits, each a gsl-vector of the samelength
   The integration uses a fixed number
   of function calls number-of-samples, and obtains random sampling points using
   the random number generator 'generator. A previously allocated workspace
   'state must be supplied.  The result of the integration is returned
   with an estimated absolute error.")

;;;;****************************************************************************
;;;; VEGAS
;;;;****************************************************************************

;;; The vegas algorithm of Lepage is based on importance sampling.  It
;;; samples points from the probability distribution described by the
;;; function |f|, so that the points are concentrated in the regions
;;; that make the largest contribution to the integral.

(defmobject monte-carlo-vegas
    "gsl_monte_vegas"
  ((dim :sizet))
  "vegas Monte Carlo integration"
  :documentation			; FDL
  "Make and initialize a workspace for Monte Carlo integration in
   dimension dim.  The workspace is used to maintain
   the state of the integration.  Returns a pointer to vegas-state."
  :initialize-suffix "init"
  :initialize-args nil)

(defmfun get-mcv-parameters (object)
  "gsl_monte_vegas_params_get"
  (((mpointer object) :pointer) (params (:pointer (:struct vegas-params))))
  :c-return :void
  :return (params)
  :documentation "Get all parameters, as a foreign struct, for the VEGAS method."
  :export nil
  :index parameter
  :gsl-version (1 13))

(defmethod parameter ((object monte-carlo-vegas) parameter)
  (cffi:foreign-slot-value (get-mcv-parameters object) 'vegas-params parameter))

(defmfun set-mcv-parameters (object params)
  "gsl_monte_vegas_params_set"
  (((mpointer object) :pointer) (params (:pointer (:struct vegas-params))))
  :c-return :void
  :return (params)
  :export nil
  :index (setf parameter)
  :gsl-version (1 13))

(defmethod (setf parameter) (value (object monte-carlo-vegas) parameter)
  (let ((current-params (get-mcv-parameters object)))
    (setf (cffi:foreign-slot-value current-params '(:struct vegas-params) parameter)
	  value)
    (set-mcv-parameters object current-params)))

;;; As of GSL v1.13, the API above is used to get/set the VEGAS parameters, so the following is removed.
#+obsolete-gsl
(export 'vegas-parameter)
#+obsolete-gsl
(defmacro vegas-parameter (workspace parameter)
  ;; FDL
  "Get or set with setf the parameter value for the VEGAS Monte Carlo
   integration method."
  ;; (vegas-parameter ws bins-max)
  ;; (setf (vegas-parameter ws bins-max) 300)
 `(cffi:foreign-slot-value ,workspace 'vegas-state ',parameter))

(defmfun monte-carlo-integrate-vegas
    (function lower-limits upper-limits 
	      &optional
	      (number-of-samples
	       (* *monte-carlo-default-samples-per-dimension*
		  (dim0 lower-limits)))
	      (generator (make-random-number-generator +mt19937+ 0))
	      (state (make-monte-carlo-vegas (dim0 lower-limits)))
	      (scalars t))
  "gsl_monte_vegas_integrate"
  ((callback :pointer)
   ((grid:foreign-pointer lower-limits) :pointer) ((grid:foreign-pointer upper-limits) :pointer)
   ((dim0 lower-limits) :sizet) (number-of-samples :sizet)
   ((mpointer generator) :pointer)
   ((mpointer state) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :inputs (lower-limits upper-limits)
  :callbacks
  (callback (:struct fnstruct-dimension) (dimension)
	    (function :double (:input :double :cvector dim0) :slug))
  :callback-dynamic (((dim0 lower-limits)) (function scalars))
  :documentation			; FDL
  "Uses the vegas Monte Carlo algorithm to integrate the function f
   over the dim-dimensional hypercubic region defined by the lower and
   upper limits in the arrays x1 and xu, each of the same length.  The
   integration uses a fixed number of function calls
   number-of-samples, and obtains random sampling points using the
   random number generator r.  A previously allocated workspace s must
   be supplied.  The result of the integration is returned with an
   estimated absolute error.  The result and its error estimate are
   based on a weighted average of independent samples. The chi-squared
   per degree of freedom for the weighted average is returned via the
   state struct component, s->chisq, and must be consistent with 1 for
   the weighted average to be reliable.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

;;; Example from Sec. 23.5

(defun mcrw (x y z)
  "Example function for Monte Carlo used in random walk studies."
  (* (/ (expt dpi 3))
     (/ (- 1 (* (cos x) (cos y) (cos z))))))

(defparameter *mc-lower*
  (grid:make-foreign-array
   'double-float :initial-contents '(0.0d0 0.0d0 0.0d0)))

(defparameter *mc-upper*
  (grid:make-foreign-array
   'double-float :initial-contents (make-list 3 :initial-element dpi)))

(defun random-walk-plain-example (&optional (nsamples 500000))
  (monte-carlo-integrate-plain 'mcrw *mc-lower* *mc-upper* nsamples))

(defun random-walk-miser-example (&optional (nsamples 500000))
  (monte-carlo-integrate-miser 'mcrw *mc-lower* *mc-upper* nsamples))

(defun random-walk-vegas-example (&optional (nsamples 500000))
  (monte-carlo-integrate-vegas 'mcrw *mc-lower* *mc-upper* nsamples))

(save-test monte-carlo
  (random-walk-plain-example)
  (random-walk-miser-example)
  (random-walk-vegas-example))
