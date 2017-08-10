;; Numerical integration techniques that require tables
;; Liam Healy 2009-04-04 15:24:05EDT 
;; Time-stamp: <2012-01-13 12:01:39EST numerical-integration-with-tables.lisp>
;;
;; Copyright 2009, 2011 Liam M. Healy
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
;;;; QAWS adaptive integration for singular functions
;;;;****************************************************************************

(defmobject qaws-table
    "gsl_integration_qaws_table"
  ((alpha :double) (beta :double) (mu :int) (nu :int))
  "table for QAWS numerical integration method"
  :documentation			; FDL
  "Make and initialize a table for the QAWS
   adaptive integration method for singular functions.
   It a singular weight function W(x) with the parameters (alpha, beta, mu, nu),
          W(x) = (x-a)^alpha (b-x)^beta log^mu (x-a) log^nu (b-x)
   where alpha > -1, beta > -1, and mu = 0, 1, nu = 0, 1. The
   weight function can take four different forms depending on the
   values of mu and nu,
          W(x) = (x-a)^alpha (b-x)^beta                   (mu = 0, nu = 0)
          W(x) = (x-a)^alpha (b-x)^beta log(x-a)          (mu = 1, nu = 0)
          W(x) = (x-a)^alpha (b-x)^beta log(b-x)          (mu = 0, nu = 1)
          W(x) = (x-a)^alpha (b-x)^beta log(x-a) log(b-x) (mu = 1, nu = 1)
   The singular points (a,b) do not have to be specified until the
   integral is computed, where they are the endpoints of the integration
   range."
  :initialize-suffix "set"
  ;; This defines the reinitialize-instance but causes the make-qaws-table
  ;; to initialize twice.
  :initialize-args ((alpha :double) (beta :double) (mu :int) (nu :int)))

(defmfun integration-QAWS
    (function a b alpha beta mu nu
	      &optional
	      (absolute-error *default-absolute-error*)
	      (relative-error *default-relative-error*)
	      (table (make-qaws-table alpha beta mu nu))
	      (limit 1000) (workspace (make-integration-workspace limit)))
  "gsl_integration_qaws"
  ((callback :pointer)
   (a :double) (b :double)
   ((mpointer table) :pointer)
   (absolute-error :double) (relative-error :double)
   (limit :sizet) ((mpointer workspace) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Compute the integral of the function f(x) over the interval (a,b)
  with the singular weight function (x-a)^alpha (b-x)^beta
  log^mu (x-a) log^nu (b-x). The parameters of the weight
  function (alpha, beta, mu, nu) are used to make the default table. The
  integral is
    I = int_a^b dx f(x) (x-a)^alpha (b-x)^beta log^mu (x-a) log^nu (b-x).
  The adaptive bisection algorithm of QAG is used. When a
  subinterval contains one of the endpoints then a special 25-point
  modified Clenshaw-Curtis rule is used to control the
  singularities. For subintervals which do not include the endpoints
  an ordinary 15-point Gauss-Kronrod integration rule is used.")

;;;;****************************************************************************
;;;; QAWO adaptive integration for oscillatory functions
;;;;****************************************************************************

(defmobject qawo-table
    "gsl_integration_qawo_table"
  ((omega :double) (L :double) (trig integrate-sine-cosine) (n :sizet))
  "table for QAWO numerical integration method"
  :documentation			; FDL
  "Make a table describing a sine or cosine weight function W(x) with
  the parameters (omega, L),
          W(x) = sin(omega x)
          W(x) = cos(omega x)
  The parameter L must be the length of the interval over which the
  function will be integrated L = b - a. The choice of sine or cosine
  is made with the parameter trig which should be one of :cosine or :sine.
  This makes a table of the trigonometric coefficients required in the
  integration process. The parameter n determines the number of levels
  of coefficients that are computed. Each level corresponds to one
  bisection of the interval L, so that n levels are sufficient for
  subintervals down to the length L/2^n.  An error of class
  'table-limit-exceeded is signalled if the number of levels is
  insufficient for the requested accuracy."
  :initialize-suffix "set"
  ;; This defines the reinitialize-instance but causes the make-qawo-table
  ;; to initialize twice.
  :initialize-args ((omega :double) (L :double) (trig integrate-sine-cosine)))

(defmfun integration-QAWO
    (function a omega L trig n
	      &optional
	      (absolute-error *default-absolute-error*)
	      (relative-error *default-relative-error*)
	      (table (make-qawo-table omega L trig n))
	      (limit 1000) (workspace (make-integration-workspace limit)))
  "gsl_integration_qawo"
  ((callback :pointer)
   (a :double)
   (absolute-error :double) (relative-error :double)
   (limit :sizet) ((mpointer workspace) :pointer)
   ((mpointer table) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "Use an adaptive algorithm to compute the integral of f over (a,b)
   with the weight function \sin(\omega x) or \cos(\omega x) defined
   by the table wf,
          I = \int_a^b dx f(x) sin(omega x)
          I = \int_a^b dx f(x) cos(omega x)

   The results are extrapolated using the epsilon-algorithm to
   accelerate the convergence of the integral. The function returns
   the final approximation from the extrapolation, result, and an
   estimate of the absolute error, abserr. The subintervals and their
   results are stored in the memory provided by workspace. The maximum
   number of subintervals is given by limit, which may not exceed the
   allocated size of the workspace.

   Those subintervals with large widths d where d\omega > 4 are
   computed using a 25-point Clenshaw-Curtis integration rule, which
   handles the oscillatory behavior. Subintervals with a small
   widths where d\omega < 4 are computed using a 15-point
   Gauss-Kronrod integration.")

;;;;****************************************************************************
;;;; QAWF adaptive integration for Fourier integrals
;;;;****************************************************************************

(defmfun integration-QAWF
    (function a omega L trig n
	      &optional
	      (absolute-error *default-absolute-error*)
	      (table (make-qawo-table omega L trig n))
	      (limit 1000)
	      (workspace (make-integration-workspace limit))
	      (cycle-workspace (make-integration-workspace limit)))
  "gsl_integration_qawf"
  ((callback :pointer)
   (a :double)
   (absolute-error :double)
   (limit :sizet)
   ((mpointer workspace) :pointer) ((mpointer cycle-workspace) :pointer)
   ((mpointer table) :pointer)
   (result (:pointer :double)) (abserr (:pointer :double)))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :callback-dynamic (nil (function))
  :documentation			; FDL
  "This function attempts to compute a Fourier integral of the
   function f over the semi-infinite interval [a,+\infty).

              I = \int_a^{+\infty} dx f(x) sin(omega x)
              I = \int_a^{+\infty} dx f(x) cos(omega x)

    The parameter \omega and choice of \sin or \cos is taken from the
    table wf (the length L can take any value, since it is overridden
    by this function to a value appropriate for the fourier
    integration). The integral is computed using the QAWO algorithm
    over each of the subintervals,

              C_1 = [a, a + c]
              C_2 = [a + c, a + 2 c]
              ... = ...
              C_k = [a + (k-1) c, a + k c]

    where c = (2 floor(|\omega|) + 1) \pi/|\omega|. The width c is
    chosen to cover an odd number of periods so that the contributions
    from the intervals alternate in sign and are monotonically
    decreasing when f is positive and monotonically decreasing. The
    sum of this sequence of contributions is accelerated using the
    epsilon-algorithm.

    This function works to an overall absolute tolerance of
    abserr. The following strategy is used: on each interval C_k the
    algorithm tries to achieve the tolerance

              TOL_k = u_k abserr

    where u_k = (1 - p)p^{k-1} and p = 9/10. The sum of the geometric
    series of contributions from each interval gives an overall
    tolerance of abserr.

    If the integration of a subinterval leads to difficulties then the
    accuracy requirement for subsequent intervals is relaxed,

              TOL_k = u_k max(abserr, max_{i<k}{E_i})

    where E_k is the estimated error on the interval C_k.

    The subintervals and their results are stored in the memory
    provided by workspace. The maximum number of subintervals is given
    by limit, which may not exceed the allocated size of the
    workspace. The integration over each subinterval uses the memory
    provided by cycle_workspace as workspace for the QAWO algorithm.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(defun integration-test-f456 (x)
  (if (zerop x)
      0.0d0
      ;; Note this function is different than the C case; it takes the
      ;; absolute value of the argument and then transfers the sign of
      ;; x to the answer.
      (* (signum x) (log (abs x)))))

(defun integration-test-f457 (x)
  (if (zerop x)
      0.0d0
      (/ (sqrt x))))

(defun integration-test-f458 (x)
  (if (zerop x)
      0.0d0
      (expt (1+ (expt (log x) 2)) -2)))

;; Tests from gsl-1.11/integration/test.c
;; Functions defined in gsl-1.11/integration/tests.c
(save-test numerical-integration
 (integration-QAWS
  'integration-test-f458 0.0d0 1.0d0 0.0d0 0.0d0 1 0 0.0d0 1.0d-7)
 (integration-QAWS
  'integration-test-f458 0.0d0 1.0d0 -0.5d0 -0.3d0 0 0 0.0d0 1.0d-7)
 (integration-QAWS
  'integration-test-f458 0.0d0 1.0d0 -0.5d0 -0.3d0 1 0 0.0d0 1.0d-7)
 (integration-QAWS
  'integration-test-f458 0.0d0 1.0d0 -0.5d0 -0.3d0 0 1 0.0d0 1.0d-7)
 (integration-QAWS
  'integration-test-f458 0.0d0 1.0d0 -0.5d0 -0.3d0 1 1 0.0d0 1.0d-7)
 ;; Thought test.c has n=1000, I find that this causes a floating
 ;; point overflow in making the table.  Setting to 100 works and
 ;; gives the result given in test.c.
 (integration-QAWO
  'integration-test-f456 0.0d0 (* 10 dpi) 1.0d0 :sine 100 0.0d0 1.0d-7)
 ;; For the negative interval, it was necessary to have the function
 ;; takethe absolute value of the argument and then transfer the sign of
 ;;  x to the answer.
 (integration-QAWO
  'integration-test-f456 0.0d0 (* 10 dpi) -1.0d0 :sine 100 0.0d0 1.0d-7)
 (integration-QAWF
  'integration-test-f457 0.0d0 (/ dpi 2) 1.0d0 :cosine 1000 1.0d-7))
