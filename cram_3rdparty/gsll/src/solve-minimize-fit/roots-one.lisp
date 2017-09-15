;; One-dimensional root solver.
;; Liam Healy 
;; Time-stamp: <2011-10-30 10:34:42EDT roots-one.lisp>
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

;;; /usr/include/gsl/gsl_roots.h

;;;;****************************************************************************
;;;; Initialization
;;;;****************************************************************************

(defmobject one-dimensional-root-solver-f "gsl_root_fsolver"
  ((type :pointer))
  "one-dimensional root solver with function only"
  :initialize-suffix "set"
  :initialize-args ((callback :pointer) (lower :double) (upper :double))
  :callbacks
  (callback (:struct fnstruct) nil (function :double (:input :double) :slug))
  :singular (function))

(defmobject one-dimensional-root-solver-fdf "gsl_root_fdfsolver"
  ((type :pointer))
  "one-dimensional root solver with function and derivative"
  :initialize-suffix "set"
  :initialize-args ((callback :pointer) (root-guess :double))
  :callbacks
  (callback (:struct fnstruct-fdf) nil
	    (function :double (:input :double) :slug)
	    (df :double (:input :double) :slug)
	    (fdf :void (:input :double) :slug
		 (:output :double :cvector 1) (:output :double :cvector 1)))
  :arglists-function
  (lambda (set)
    `((type &optional (function nil ,set) df fdf root-guess)
      (:type type)
      (:functions (list function df fdf) :root-guess root-guess))))

(defmfun name ((solver one-dimensional-root-solver-f))
  "gsl_root_fsolver_name"
  (((mpointer solver) :pointer))
  :definition :method
  :c-return :string
  :documentation			; FDL
  "The name of the solver.")

(defmfun name ((solver one-dimensional-root-solver-fdf))
  "gsl_root_fdfsolver_name"
  (((mpointer solver) :pointer))
  :definition :method
  :c-return :string
  :documentation			; FDL
  "The name of the solver.")

;;;;****************************************************************************
;;;; Iteration
;;;;****************************************************************************

;; It appears that this is always returning :SUCCESS (0).
(defmfun iterate ((solver one-dimensional-root-solver-f))
  "gsl_root_fsolver_iterate"
  (((mpointer solver) :pointer))
  :definition :method
  :callback-object solver
  :documentation			; FDL
  "Perform a single iteration of the solver.  The following errors may
   be signalled: 'bad-function-supplied, the iteration encountered a
   singular point where the function or its derivative evaluated to
   infinity or NaN, or 'gsl-division-by-zero, the derivative of the
   function vanished at the iteration point, preventing the algorithm
   from continuing without a division by zero.")

(defmfun iterate ((solver one-dimensional-root-solver-fdf))
  "gsl_root_fdfsolver_iterate"
  (((mpointer solver) :pointer))
  :definition :method
  :callback-object solver
  :documentation			; FDL
  "Perform a single iteration of the solver.  The following errors may
   be signalled: 'bad-function-supplied, the iteration encountered a
   singular point where the function or its derivative evaluated to
   infinity or NaN, or 'gsl-division-by-zero, the derivative of the
   function vanished at the iteration point, preventing the algorithm
   from continuing without a division by zero.")

(defmfun solution ((solver one-dimensional-root-solver-f))
  "gsl_root_fsolver_root"
  (((mpointer solver) :pointer))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "The current estimate of the root for the solver.")

(defmfun solution ((solver one-dimensional-root-solver-fdf))
  "gsl_root_fdfsolver_root"
  (((mpointer solver) :pointer))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "The current estimate of the root for the solver.")

(defmfun fsolver-lower (solver)
  "gsl_root_fsolver_x_lower"
  (((mpointer solver) :pointer))
  :c-return :double
  :documentation			; FDL
  "The lower end of the current bracketing interval for the solver.")

(defmfun fsolver-upper (solver)
  "gsl_root_fsolver_x_upper"
  (((mpointer solver) :pointer))
  :c-return :double
  :documentation 			; FDL
  "The upper end of the current bracketing interval for the solver.")

;;;;****************************************************************************
;;;; Search stopping conditions
;;;;****************************************************************************

(defmfun root-test-interval (lower upper absolute-error relative-error)
  "gsl_root_test_interval"
  ((lower :double) (upper :double)
   (absolute-error :double) (relative-error :double))
  :c-return :success-continue	 ; GSL documentation not clear on this
  :documentation			; FDL
  "Test for the convergence of the interval [lower,upper]
   with absolute error absolute-error and relative error
   relative-error.  This returns T
   if the following condition is achieved,
   |a - b| < epsabs + epsrel min(|a|,|b|) 
   when the interval x = [a,b] does not include the origin.  If the
   interval includes the origin then min(|a|,|b|) is replaced by
   zero (which is the minimum value of |x| over the interval).  This
   ensures that the relative error is accurately estimated for roots close
   to the origin.

   This condition on the interval also implies that any estimate of the
   root r in the interval satisfies the same condition with respect
   to the true root r^*, |r - r^*| < epsabs + epsrel r^*
   assuming that the true root r^* is contained within the interval.")

(defmfun root-test-delta (x1 x0 absolute-error relative-error)
  "gsl_root_test_delta"
  ((x1 :double) (x0 :double)
   (absolute-error :double) (relative-error :double))
  :c-return :success-continue
  :documentation			; FDL
  "Test for the convergence of the sequence ... x0, x1
   with absolute error absolute-error and relative error
   relative-error.  The test returns T if the following
   condition is achieved,
   |x_1 - x_0| < epsabs + epsrel |x_1|
   and returns NIL otherwise.")

(defmfun root-test-residual (f absolute-error)
  "gsl_root_test_residual"
  ((f :double) (absolute-error :double))
  :c-return :success-continue
  :documentation			; FDL
  "Tests the residual value f against the absolute
   error bound absolute-error.  The test returns T if the
   following condition is achieved,
   |f| < epsabs
   and returns NIL otherwise.  This criterion is suitable
   for situations where the precise location of the root, x, is
   unimportant provided a value can be found where the residual,
   |f(x)|, is small enough.")

;;;;****************************************************************************
;;;; Root bracketing algorithms
;;;;****************************************************************************

(defmpar +bisection-fsolver+ "gsl_root_fsolver_bisection"
  ;; FDL
  "The bisection algorithm is the simplest method of bracketing the
   roots of a function.   It is the slowest algorithm provided by
   the library, with linear convergence.

   On each iteration, the interval is bisected and the value of the
   function at the midpoint is calculated.  The sign of this value is used
   to determine which half of the interval does not contain a root.  That
   half is discarded to give a new, smaller interval containing the
   root.  This procedure can be continued indefinitely until the interval is
   sufficiently small.

   At any time the current estimate of the root is taken as the midpoint of
   the interval.")

(defmpar +false-position-fsolver+ "gsl_root_fsolver_falsepos"
  ;; FDL
  "The false position algorithm is a method of finding roots based on
   linear interpolation.  Its convergence is linear, but it is usually
   faster than bisection.

   On each iteration a line is drawn between the endpoints (a,f(a))
   and (b,f(b)) and the point where this line crosses the
   x-axis taken as a ``midpoint''.  The value of the function at
   this point is calculated and its sign is used to determine which side of
   the interval does not contain a root.  That side is discarded to give a
   new, smaller interval containing the root.  This procedure can be
   continued indefinitely until the interval is sufficiently small.

   The best estimate of the root is taken from the linear interpolation of
   the interval on the current iteration.")

(defmpar +brent-fsolver+ "gsl_root_fsolver_brent"
  ;; FDL
  "The Brent-Dekker method (referred to here as Brent's method)
   combines an interpolation strategy with the bisection algorithm.  This
   produces a fast algorithm which is still robust.

   On each iteration Brent's method approximates the function using an
   interpolating curve.  On the first iteration this is a linear
   interpolation of the two endpoints.  For subsequent iterations the
   algorithm uses an inverse quadratic fit to the last three points, for
   higher accuracy.  The intercept of the interpolating curve with the
   x-axis is taken as a guess for the root.  If it lies within the
   bounds of the current interval then the interpolating point is accepted,
   and used to generate a smaller interval.  If the interpolating point is
   not accepted then the algorithm falls back to an ordinary bisection
   step.

   The best estimate of the root is taken from the most recent
   interpolation or bisection.")

;;;;****************************************************************************
;;;; Root finding algorithms using derivatives
;;;;****************************************************************************

(defmpar +newton-fdfsolver+ "gsl_root_fdfsolver_newton"
  ;; FDL
  "Newton's Method is the standard root-polishing algorithm.  The algorithm
   begins with an initial guess for the location of the root.  On each
   iteration, a line tangent to the function f is drawn at that
   position.  The point where this line crosses the x-axis becomes
   the new guess.  The iteration is defined by the following sequence,
   x_{i+1} = x_i - f(x_i) / f'(x_i)
   Newton's method converges quadratically for single roots, and linearly
   for multiple roots.")

(defmpar +secant-fdfsolver+ "gsl_root_fdfsolver_secant"
  ;; FDL
  "The secant method is a simplified version of Newton's method which does
   not require the computation of the derivative on every step.
   On its first iteration the algorithm begins with Newton's method, using
   the derivative to compute a first step,
   x_1 = x_0 - f(x_0)/f'(x_0)
   Subsequent iterations avoid the evaluation of the derivative by
   replacing it with a numerical estimate, the slope of the line through
   the previous two points,
   x_{i+1} = x_i - f(x_i) / f'_{est}
    where
   f'_{est} =  f(x_{i}) - f(x_{i-1}) / x_i - x_{i-1}
   When the derivative does not change significantly in the vicinity of the
   root the secant method gives a useful saving.  Asymptotically the secant
   method is faster than Newton's method whenever the cost of evaluating
   the derivative is more than 0.44 times the cost of evaluating the
   function itself.  As with all methods of computing a numerical
   derivative the estimate can suffer from cancellation errors if the
   separation of the points becomes too small.

   On single roots, the method has a convergence of order (1 + \sqrt
   5)/2 (approximately 1.62).  It converges linearly for multiple
   roots.")

(defmpar +steffenson-fdfsolver+ "gsl_root_fdfsolver_steffenson"
  ;; FDL
  "The Steffenson method provides the fastest convergence of all the
   routines.  It combines the basic Newton algorithm with an Aitken
   ``delta-squared'' acceleration.  If the Newton iterates are x_i
   then the acceleration procedure generates a new sequence R_i,
   R_i = x_i - (x_{i+1} - x_i)^2 / (x_{i+2} - 2 x_{i+1} + x_i)
   which converges faster than the original sequence under reasonable
   conditions.  The new sequence requires three terms before it can produce
   its first value so the method returns accelerated values on the second
   and subsequent iterations.  On the first iteration it returns the
   ordinary Newton estimate.  The Newton iterate is also returned if the
   denominator of the acceleration term ever becomes zero.

   As with all acceleration procedures this method can become unstable if
   the function is not well-behaved.")

;;;;****************************************************************************
;;;; Examples
;;;;****************************************************************************

;;; This is the example given in Sec. 32.10.

(let ((a 1.0d0) (b 0.0d0) (c -5.0d0))
  (defun quadratic (x)
    (+ (* (+ (* a x) b) x) c))
  (defun quadratic-derivative (x)
    (+ (* 2 a x) b))
  (defun quadratic-and-derivative (x)
    (values (+ (* (+ (* a x) b) x) c)
	    (+ (* 2 a x) b))))

(defun roots-one-example-no-derivative
    (&optional (method +brent-fsolver+) (print-steps t))
  "Solving a quadratic, the example given in Sec. 32.10 of the GSL manual."
  (let ((max-iter 50)
	(solver
	 (make-one-dimensional-root-solver-f method 'quadratic 0.0d0 5.0d0)))
    (when print-steps
      (format t "iter ~6t   [lower ~24tupper] ~36troot ~44terr ~54terr(est)~&"))
    (loop for iter from 0
       for root = (solution solver)
       for lower = (fsolver-lower solver)
       for upper = (fsolver-upper solver)
       do (iterate solver)
       while  (and (< iter max-iter)
		   (not (root-test-interval lower upper 0.0d0 0.001d0)))
       do
       (when print-steps
	 (format t "~d~6t~10,6f~18t~10,6f~28t~12,9f ~44t~10,4g ~10,4g~&"
		 iter lower upper
		 root (- root (sqrt 5.0d0))
		 (- upper lower)))
       finally (return root))))

(defun roots-one-example-derivative
    (&optional (method +newton-fdfsolver+) (print-steps t))
  "Solving a quadratic, the example given in Sec. 32.10 of the GSL manual."
  (let* ((max-iter 100)
	 (initial 5.0d0)
	 (solver (make-one-dimensional-root-solver-fdf
		  method
		  'quadratic 'quadratic-derivative 'quadratic-and-derivative
		  initial)))
    (when print-steps
      (format t "iter ~6t ~8troot ~22terr ~34terr(est)~&"))
    (loop for iter from 0
       for oldroot = initial then root
       for root = (progn (iterate solver) (solution solver))
       while (and (< iter max-iter)
		  (not (root-test-delta root oldroot 0.0d0 1.0d-5)))
       do
       (when print-steps
	 (format t "~d~6t~10,8g ~18t~10,6g~34t~10,6g~&"
		 iter root (- root (sqrt 5.0d0)) (- root oldroot)))
       finally (return root))))

;; To see step-by-step information as the solution progresses, make
;; the last argument T.
(save-test roots-one
 (roots-one-example-no-derivative +bisection-fsolver+ nil)
 (roots-one-example-no-derivative +false-position-fsolver+ nil)
 (roots-one-example-no-derivative +brent-fsolver+ nil)
 (roots-one-example-derivative +newton-fdfsolver+ nil)
 (roots-one-example-derivative +secant-fdfsolver+ nil)
 (roots-one-example-derivative +steffenson-fdfsolver+ nil))
