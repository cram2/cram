;; Numerical integration
;; Liam Healy, Wed Jul  5 2006 - 23:14
;; Time-stamp: <2015-01-22 23:32:29EST numerical-integration.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2010, 2011, 2012, 2015 Liam M. Healy
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

;;; /usr/include/gsl/gsl_integration.h

;;;;****************************************************************************
;;;; Default error values
;;;;****************************************************************************

(export '(*default-absolute-error* *default-relative-error*))

(defparameter *default-absolute-error* 1.0d-5
  "The default absolute error used in numerical integration.")

(defparameter *default-relative-error* 0.0d0
  "The default relative error used in numerical integration.")

;;;;****************************************************************************
;;;; QNG non-adaptive Gauss-Kronrod integration
;;;;****************************************************************************

(defmfun integration-QNG
    (function a b
	      &optional (absolute-error *default-absolute-error*)
	      (relative-error *default-relative-error*))
  ;; Set absolute-error and relative-error to 1 because it apparently doesn't matter
  ;; what these are if they are too large, it will do a minimum number
  ;; of points anyway.
  "gsl_integration_qng"
  ((callback :pointer)
   (a :double) (b :double)
   (absolute-error :double) (relative-error :double)
   (result (:pointer :double)) (abserr (:pointer :double))
   (neval (:pointer :sizet)))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Apply the Gauss-Kronrod 10-point, 21-point, 43-point and
   87-point integration rules in succession until an estimate of the
   integral of f over (a,b) is achieved within the desired
   absolute and relative error limits, absolute-error and relative-error.  The
   function returns the final approximation, an estimate of
   the absolute error, and the number of function evaluations
   used.  The Gauss-Kronrod rules are designed in such a way
   that each rule uses all the results of its predecessors, in order to
   minimize the total number of function evaluations.")

;;;;****************************************************************************
;;;; QAG adaptive Gauss-Kronrod integration
;;;;****************************************************************************

(defmobject integration-workspace
    "gsl_integration_workspace" ((size :sizet))
    "integration workspace"
    :documentation			; FDL
    "Make a workspace sufficient to hold n double
     precision intervals, their integration results and error estimates.")

(defmfun integration-QAG
    (function a b method
	      &optional
	      (absolute-error *default-absolute-error*)
	      (relative-error *default-relative-error*)
	      (limit 1000) (workspace (make-integration-workspace limit)))
  ;; Set absolute-error and relative-error to 1 because it apparently doesn't matter
  ;; what these are if they are too large, it will do a minimum number
  ;; of points anyway.
  "gsl_integration_qag"
  ((callback :pointer)
   (a :double) (b :double)
   (absolute-error :double) (relative-error :double)
   (limit :sizet) (method integrate-method) ((mpointer workspace) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Apply an integration rule adaptively until an estimate
  of the integral of f over (a,b) is achieved within the
  desired absolute and relative error limits, absolute-error and
  relative-error.  The function returns the final approximation,
  and an estimate of the absolute error.  The integration rule
  is determined by the value of method, which should
  be chosen from the following symbolic names,
  :gauss15 :gauss21 :gauss31 :gauss41 :gauss51 :gauss61
  corresponding to the 15, 21, 31, 41, 51 and 61 point Gauss-Kronrod
  rules.  The higher-order rules give better accuracy for smooth functions,
  while lower-order rules save time when the function contains local
  difficulties, such as discontinuities.
  On each iteration the adaptive integration strategy bisects the interval
  with the largest error estimate.  The subintervals and their results are
  stored in the memory provided by workspace.  The maximum number of
  subintervals is given by 'limit, which may not exceed the allocated
  size of the workspace.")

;;;;****************************************************************************
;;;; QAGS adaptive integration with singularity
;;;;****************************************************************************

(defmfun integration-QAGS
    (function a b 
	      &optional (absolute-error *default-absolute-error*)
	      (relative-error *default-relative-error*)
	      (limit 1000) (workspace (make-integration-workspace limit)))
  "gsl_integration_qags"
  ((callback :pointer)
   (a :double) (b :double)
   (absolute-error :double) (relative-error :double) (limit :sizet)
   ((mpointer workspace) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Apply the Gauss-Kronrod 21-point integration rule
   adaptively until an estimate of the integral of f over
   (a,b) is achieved within the desired absolute and relative error
   limits, absolute-error and relative-error.  The results are extrapolated
   using the epsilon-algorithm, which accelerates the convergence of the
   integral in the presence of discontinuities and integrable
   singularities.  The function returns the final approximation from the
   extrapolation, and an estimate of the absolute error.  The subintervals
   and their results are stored in the
   memory provided by workspace.  The maximum number of subintervals
   is given by limit, which may not exceed the allocated size of the
   workspace.")

;;;;****************************************************************************
;;;; QAGP adaptive integration with known singular points
;;;;****************************************************************************

(defmfun integration-QAGP
    (function points
	      &optional
	      (absolute-error *default-absolute-error*)
	      (relative-error *default-relative-error*)
	      (limit 1000) (workspace (make-integration-workspace limit)))
  "gsl_integration_qagp"
  ((callback :pointer)
   ((grid:foreign-pointer points) :pointer) ((dim0 points) :sizet)
   (absolute-error :double) (relative-error :double) (limit :sizet)
   ((mpointer workspace) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :inputs (points)
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Apply the adaptive integration algorithm QAGS taking
   account of the user-supplied locations of singular points.  The array
   points should contain the endpoints of the
   integration ranges defined by the integration region and locations of
   the singularities.  For example, to integrate over the region
   (a,b) with break-points at x_1, x_2, x_3 (where 
   a < x_1 < x_2 < x_3 < b) then an array with
   (setf (data array) #(a x_1 x_2 x_3 b)) should be used.
   If you know the locations of the singular points in the integration
   region then this routine will be faster than #'integration-QAGS.")

;;;;****************************************************************************
;;;; QAGI adaptive integration on infinite intervals
;;;;****************************************************************************

(defmfun integration-QAGi
    (function 
     &optional
     (absolute-error *default-absolute-error*)
     (relative-error *default-relative-error*)
     (limit 1000) (workspace (make-integration-workspace limit)))
  "gsl_integration_qagi"
  ((callback :pointer)
   (absolute-error :double) (relative-error :double) (limit :sizet)
   ((mpointer workspace) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Compute the integral of the function f over the
   infinite interval (-\infty,+\infty).  The integral is mapped onto the
   semi-open interval (0,1] using the transformation x = (1-t)/t,
   \int_{-\infty}^{+\infty} dx \, f(x) 
    = \int_0^1 dt \, (f((1-t)/t) + f(-(1-t)/t))/t^2.
   It is then integrated using the QAGS algorithm.  The normal 21-point
   Gauss-Kronrod rule of QAGS is replaced by a 15-point rule, because the
   transformation can generate an integrable singularity at the origin.  In
   this case a lower-order rule is more efficient.")

(defmfun integration-QAGiu
    (function a
	      &optional
	      (absolute-error *default-absolute-error*)
	      (relative-error *default-relative-error*)
	      (limit 1000) (workspace (make-integration-workspace limit)))
  "gsl_integration_qagiu"
  ((callback :pointer) (a :double)
   (absolute-error :double) (relative-error :double) (limit :sizet)
   ((mpointer workspace) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Compute the integral of the function f over the
   semi-infinite interval (a,+\infty).  The integral is mapped onto the
   semi-open interval (0,1] using the transformation x = a + (1-t)/t,
   int_{a}^{+\infty} dx,  f(x) = \int_0^1 dt f(a + (1-t)/t)/t^2
   and then integrated using the QAGS algorithm.")

(defmfun integration-QAGil
    (function b
	      &optional
	      (absolute-error *default-absolute-error*)
	      (relative-error *default-relative-error*)
	      (limit 1000) (workspace (make-integration-workspace limit)))
  "gsl_integration_qagil"
  ((callback :pointer) (b :double)
   (absolute-error :double) (relative-error :double) (limit :sizet)
   ((mpointer workspace) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Compute the integral of the function f over the
   semi-infinite interval (-\infty,b).  The integral is mapped onto the
   semi-open interval (0,1] using the transformation x = b - (1-t)/t,
   \int_{-\infty}^{b} dx, f(x) = \int_0^1 dt, f(b - (1-t)/t)/t^2
   and then integrated using the QAGS algorithm.")

;;;;****************************************************************************
;;;; QAWC adaptive integration for Cauchy principal values
;;;;****************************************************************************

(defmfun integration-QAWC
    (function a b c 
	      &optional
	      (absolute-error *default-absolute-error*)
	      (relative-error *default-relative-error*)
	      (limit 1000) (workspace (make-integration-workspace limit)))
  "gsl_integration_qawc"
  ((callback :pointer)
   (a :double) (b :double) (c :double)
   (absolute-error :double) (relative-error :double) (limit :sizet)
   ((mpointer workspace) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Compute the Cauchy principal value of the integral of
   f over (a,b), with a singularity at c,
   I = \int_a^b dx, {f(x)/x - c} = lim_{epsilon -> 0}
   {\int_a^{c-epsilon} dx, {f(x)/x - c} + \int_{c+epsilon}^b dx,
   {f(x) \over x - c}}
   The adaptive bisection algorithm of QAG is used, with modifications to
   ensure that subdivisions do not occur at the singular point x = c.
   When a subinterval contains the point x = c or is close to
   it then a special 25-point modified Clenshaw-Curtis rule is used to control
   the singularity.  Further away from the singularity the algorithm
   uses an ordinary 15-point Gauss-Kronrod integration rule.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(defun integration-test-f1 (alpha)
  (lambda (x) (* (expt x alpha) (log (/ x)))))

(defun integration-test-f3 (alpha)
  (lambda (x) (cos (* (expt 2 alpha) (sin x)))))

(defun integration-test-f11 (alpha)
  (lambda (x) (realpart (expt (log (/ x)) (1- alpha)))))

(defun integration-test-f15 (alpha)
  (lambda (x) (* x x (exp (* (- (expt 2 (- alpha))) x)))))

(defun integration-test-f16 (alpha)
  (lambda (x)
    (cond
      ((and (= alpha 1.0d0) (zerop x)) 1.0d0)
      ((and (> alpha 1.0d0) (zerop x)) 0.0d0)
      (t (realpart
	  (/ (expt x (1- alpha))
	     (expt (1+ (* 10 x)) 2)))))))

(defun integration-test-f454 (x)
  (* (expt x 3) (* (log (abs (* (- (expt x 2) 1.0d0) (- (expt x 2) 2.0d0)))))))

(defun integration-test-f455 (x)
  (/ (log x) (1+ (* 100 x x))))

(defun integration-test-f459 (x)
  (/ (+ (* 5.0d0 (expt x 3)) 6.0d0)))

(defun integration-test-myfn1 (x)
  (exp (- (- x) (expt x 2))))

(defun integration-test-myfn2 (alpha)
  (lambda (x) (exp (* alpha x))))

(save-test numerical-integration
 (integration-qng 'sin 0.0d0 dpi)
 (integration-QAG 'sin 0.0d0 dpi :gauss15 20)
 (integration-QAG 'sin 0.0d0 dpi :gauss21 40)
 ;; Tests from gsl-1.11/integration/test.c
 ;; Functions defined in gsl-1.11/integration/tests.c
 (integration-QNG (integration-test-f1 2.6d0) 0.0d0 1.0d0 0.1d0 0.0d0)
 (integration-QNG (integration-test-f1 2.6d0) 1.0d0 0.0d0 0.1d0 0.0d0)
 (integration-QNG (integration-test-f1 2.6d0) 0.0d0 1.0d0 0.0d0 1.0d-9)
 (integration-QNG (integration-test-f1 2.6d0) 1.0d0 0.0d0 0.0d0 1.0d-9)
 (integration-QNG (integration-test-f3 1.3d0) 0.3d0 2.71d0 0.0d0 1d-12)
 (integration-QNG (integration-test-f3 1.3d0) 2.71d0 0.3d0 0.0d0 1d-12)
 (integration-QNG (integration-test-f1 2.6d0) 0.0d0 1.0d0 0.0d0 1.0d-13)
 (integration-QNG (integration-test-f1 2.6d0) 1.0d0 0.0d0 0.0d0 1.0d-13)
 (integration-QNG (integration-test-f1 -0.9d0) 0.0d0 1.0d0 0.0d0 1.0d-3) ; error
 (integration-QNG (integration-test-f1 -0.9d0) 1.0d0 0.0d0 0.0d0 1.0d-3) ; error
 (integration-QAG
  (integration-test-f1 2.6d0) 0.0d0 1.0d0 :gauss15 0.0d0 1.0d-10 1000)
 (integration-QAG
  (integration-test-f1 2.6d0) 1.0d0 0.0d0 :gauss15 0.0d0 1.0d-10 1000)
 (integration-QAG
  (integration-test-f1 2.6d0) 0.0d0 1.0d0 :gauss21 1.0d-14 0.0d0 1000)
 (integration-QAG
  (integration-test-f1 2.6d0) 1.0d0 0.0d0 :gauss21 1.0d-14 0.0d0 1000)
 (integration-QAG			; roundoff error
  (integration-test-f3 1.3d0) 0.3d0 2.71d0 :gauss31 1.0d-14 0.0d0 1000)
 (integration-QAG			; roundoff error
  (integration-test-f3 1.3d0) 2.71d0 0.3d0 :gauss31 1.0d-14 0.0d0 1000)
 (integration-QAG			; singularity error
  (integration-test-f16 2.0d0) -1.0d0 1.0d0 :gauss51 1.0d-14 0.0d0 1000)
 (integration-QAG			; singularity error
  (integration-test-f16 2.0d0) 1.0d0 -1.0d0 :gauss51 1.0d-14 0.0d0 1000)
 (integration-QAG			; iteration limit error
  (integration-test-f16 2.0d0) -1.0d0 1.0d0 :gauss61 1.0d-14 0.0d0 3)
 (integration-QAG			; iteration limit error
  (integration-test-f16 2.0d0) 1.0d0 -1.0d0 :gauss61 1.0d-14 0.0d0 3)
 (integration-QAGS (integration-test-f1 2.6d0) 0.0d0 1.0d0 0.0d0 1d-10 1000)
 (integration-QAGS (integration-test-f1 2.6d0) 1.0d0 0.0d0 0.0d0 1d-10 1000)
 (integration-QAGS (integration-test-f11 2.0d0) 1.0d0 1000.0d0 1d-7 0.0d0 1000)
 (integration-QAGS (integration-test-f11 2.0d0) 1000.0d0 1.0d0 1d-7 0.0d0 1000)
 (integration-QAGiu 'integration-test-f455 0.0d0 0.0d0 1d-3 1000)
 (integration-QAGiu (integration-test-f15 5.0d0) 0.0d0 0.0d0 1d-7 1000)
 (integration-QAGiu (integration-test-f16 1.0d0) 99.9d0 1d-7 0.0d0 1000)
 (integration-QAGi 'integration-test-myfn1 1.0d-7 0.0d0 1000)
 (integration-QAGil (integration-test-myfn2 1.0d0) 1.0d0 1.0d-7 0.0d0 1000)
 (integration-QAGp
  'integration-test-f454
  (grid:copy-to (vector 0.0d0 1.0d0 (sqrt 2.0d0) 3.0d0))
  0.0d0 1.0d-3 1000)
 (integration-QAWc 'integration-test-f459 -1.0d0 5.0d0 0.0d0 0.0d0 1.0d-3 1000)
 ;; Check that integration-QAG is reentrant, but note this is not the
 ;; recommended way to do multivariate integration (see Monte Carlo).
 (integration-QAG (lambda (x)
		    (integration-QAG (lambda (y) (* (sin x) y)) 0d0 1d0 :gauss41))
		  0d0 pi :gauss41))
