;;; Multivariate roots.                
;;; Liam Healy 2008-01-12 12:49:08
;;; Time-stamp: <2010-07-13 09:52:54EDT roots-multi.lisp>
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

(in-package :gsl)

;;; /usr/include/gsl/gsl_multiroots.h

;;; Currently, functions defined for root solving will be passed
;;; scalars and should return scalars as multiple values.  A possible
;;; future enhancement is to optionally pass grid:foreign-arrays and return
;;; grid:foreign-arrays instead.  This would allow directly manipulation of
;;; grid:foreign-arrays by the user function.  Notes Mon Jan 19 2009.

;;;;****************************************************************************
;;;; Initialization
;;;;****************************************************************************

(defmobject multi-dimensional-root-solver-f "gsl_multiroot_fsolver"
  ((type :pointer) ((first dimensions) sizet))
  "multi-dimensional root solver with function only"
  :documentation			; FDL
  "Make an instance of a solver of the type specified for a system of
   the specified number of dimensions.  Optionally
   set or reset an existing solver to use the function and the
   initial guess gsl-vector.  If scalarsp is T, the functions will
   be supplied scalars, and should return scalars."
  :initialize-suffix "set"
  :initialize-args ((callback :pointer) ((mpointer initial) :pointer))
  :callbacks
  (callback fnstruct-dimension (dimension)
	    (function
	     :success-failure
	     (:input :double :foreign-array dim0) :slug
	     (:output :double :foreign-array dim0)))
  :arglists-function
  (lambda (set)
    `((type &optional function-or-dimension (initial nil ,set) (scalarsp t))
      (:type type
	     :dimensions
	     (if ,set (dimensions initial) function-or-dimension))
      (:functions
       (list function-or-dimension) :initial initial :scalarsp scalarsp)))
  :inputs (initial))

(defmobject multi-dimensional-root-solver-fdf "gsl_multiroot_fdfsolver"
  ((type :pointer) ((first dimensions) sizet))
  "multi-dimensional root solver with function and derivative"
  :documentation			; FDL
  "Make an instance of a derivative solver of the type specified for
   a system of the specified number of dimensions.  Optionally
   set or reset an existing solver to use the function and derivative
   (fdf) and the initial guess.  If scalarsp is T, the functions will
   be supplied, and should return scalars."
  :initialize-suffix "set"
  :initialize-args ((callback :pointer) ((mpointer initial) :pointer))
  :callbacks
  (callback fnstruct-dimension-fdf (dimension)
	    (function :success-failure
		      (:input :double :foreign-array dim0)
		      :slug
		      (:output :double :foreign-array dim0))
	    (df :success-failure
		(:input :double :foreign-array dim0)
		:slug
		(:output :double :foreign-array dim0 dim0))
	    (fdf :success-failure
		 (:input :double :foreign-array dim0)
		 :slug
		 (:output :double :foreign-array dim0)
		 (:output :double :foreign-array dim0 dim0)))
  :arglists-function
  (lambda (set)
    `((type &optional function-or-dimension (initial nil ,set) 
	    (scalarsp t))
      (:type type
	     :dimensions
	     (if ,set (dimensions initial) function-or-dimension))
      (:functions function-or-dimension :initial initial :scalarsp scalarsp)))
  :inputs (initial))

(defmfun name ((solver multi-dimensional-root-solver-f))
  "gsl_multiroot_fsolver_name"
  (((mpointer solver) :pointer))
  :definition :method
  :c-return :string
  :documentation			; FDL
  "The name of the solver.")

(defmfun name ((solver multi-dimensional-root-solver-fdf))
  "gsl_multiroot_fdfsolver_name"
  (((mpointer solver) :pointer))
  :definition :method
  :c-return :string
  :documentation			; FDL
  "The name of the solver.")

;;;;****************************************************************************
;;;; Iteration
;;;;****************************************************************************

(defmfun iterate ((solver multi-dimensional-root-solver-f))
  "gsl_multiroot_fsolver_iterate"
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

(defmfun iterate ((solver multi-dimensional-root-solver-fdf))
  "gsl_multiroot_fdfsolver_iterate"
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

(defmfun solution ((solver multi-dimensional-root-solver-f))
  "gsl_multiroot_fsolver_root"
  (((mpointer solver) :pointer))
  :definition :method
  :callback-object solver
  :c-return (crtn :pointer)
  :return ((make-foreign-array-from-mpointer crtn))
  :documentation			; FDL
  "The current estimate of the root for the solver.")

(defmfun solution ((solver multi-dimensional-root-solver-fdf))
  "gsl_multiroot_fdfsolver_root"
  (((mpointer solver) :pointer))
  :definition :method
  :callback-object solver
  :c-return (crtn :pointer)
  :return ((make-foreign-array-from-mpointer crtn))
  :documentation
  "The current estimate of the root for the solver.")

(defmfun function-value ((solver multi-dimensional-root-solver-f))
  "gsl_multiroot_fsolver_f"
  (((mpointer solver) :pointer))
  :definition :method
  :c-return (crtn :pointer)
  :return ((make-foreign-array-from-mpointer crtn))
  :documentation			; FDL
  "The function value f(x) at the current estimate x of the root for the solver.")

(defmfun function-value ((solver multi-dimensional-root-solver-fdf))
  "gsl_multiroot_fdfsolver_f"
  (((mpointer solver) :pointer))
  :definition :method
  :c-return (crtn :pointer)
  :return ((make-foreign-array-from-mpointer crtn))
  :documentation			; FDL
  "The function value f(x) at the current estimate x of the root for the solver.")

(defmfun last-step ((solver multi-dimensional-root-solver-f))
  "gsl_multiroot_fsolver_dx"
  (((mpointer solver) :pointer))
  :definition :method
  :c-return (crtn :pointer)
  :return ((make-foreign-array-from-mpointer crtn))
  :documentation			; FDL
  "The last step dx taken by the solver.")

(defmfun last-step ((solver multi-dimensional-root-solver-fdf))
  "gsl_multiroot_fsolver_dx"
  (((mpointer solver) :pointer))
  :definition :method
  :c-return (crtn :pointer)
  :return ((make-foreign-array-from-mpointer crtn))
  :documentation			; FDL
  "The last step dx taken by the solver.")

;;;;****************************************************************************
;;;; Search stopping conditions
;;;;****************************************************************************

;;; The only place we need to pick apart the gsl_multiroot_fsolver
;;; struct is here.  We could use last-step etc., but then we'd have
;;; to discriminate on mfsolver vs. mfdfsolver.

(defun multiroot-slot (solver slot)
  (cffi:foreign-slot-value (mpointer solver) 'gsl-multiroot-fsolver slot))

(defmfun multiroot-test-delta (solver absolute-error relative-error)
  "gsl_multiroot_test_delta"
  (((multiroot-slot solver 'dx) :pointer)
   ((multiroot-slot solver 'x) :pointer)
   (absolute-error :double) (relative-error :double))
  :c-return :success-continue
  :documentation			; FDL
  "Test for the convergence of the sequence by comparing the
   last step dx with the absolute error and relative
   errors given to the current position x.  The test returns
   T if the following condition is achieved:
   |dx_i| < epsabs + epsrel |x_i|
   for each component of x and returns NIL otherwise.")

(defmfun multiroot-test-residual (solver absolute-error)
  "gsl_multiroot_test_residual"
  (((multiroot-slot solver 'f) :pointer) (absolute-error :double))
  :c-return :success-continue
  :documentation			; FDL
  "Test the residual value f against the absolute error,
   returning T if the following condition is achieved:
   \sum_i |f_i| < absolute_error
   and returns NIL otherwise.  This criterion is suitable
   for situations where the precise location of the root x is
   unimportant provided a value can be found where the
   residual is small enough.")

;;;;****************************************************************************
;;;; Algorithms using derivatives
;;;;****************************************************************************

(defmpar +powells-hybrid+ "gsl_multiroot_fdfsolver_hybridsj"
  ;; FDL
  "This is a modified version of Powell's Hybrid method as implemented in
   the hybrj algorithm in minpack.  Minpack was written by Jorge
   J. More, Burton S. Garbow and Kenneth E. Hillstrom.  The Hybrid
   algorithm retains the fast convergence of Newton's method but will also
   reduce the residual when Newton's method is unreliable. 

   The algorithm uses a generalized trust region to keep each step under
   control.  In order to be accepted a proposed new position x' must
   satisfy the condition |D (x' - x)| < \delta, where D is a
   diagonal scaling matrix and \delta is the size of the trust
   region.  The components of D are computed internally, using the
   column norms of the Jacobian to estimate the sensitivity of the residual
   to each component of x.  This improves the behavior of the
   algorithm for badly scaled functions.

   On each iteration the algorithm first determines the standard Newton
   step by solving the system J dx = - f.  If this step falls inside
   the trust region it is used as a trial step in the next stage.  If not,
   the algorithm uses the linear combination of the Newton and gradient
   directions which is predicted to minimize the norm of the function while
   staying inside the trust region,
   dx = - \alpha J^{-1} f(x) - \beta \nabla |f(x)|^2.
   This combination of Newton and gradient directions is referred to as a
   dogleg step.

   The proposed step is now tested by evaluating the function at the
   resulting point, x'.  If the step reduces the norm of the function
   sufficiently then it is accepted and size of the trust region is
   increased.  If the proposed step fails to improve the solution then the
   size of the trust region is decreased and another trial step is
   computed.

   The speed of the algorithm is increased by computing the changes to the
   Jacobian approximately, using a rank-1 update.  If two successive
   attempts fail to reduce the residual then the full Jacobian is
   recomputed.  The algorithm also monitors the progress of the solution
   and returns an error if several steps fail to make any improvement,
   'no-progress
   the iteration is not making any progress, preventing the algorithm from
   continuing.
   'jacobian-not-improving
   re-evaluations of the Jacobian indicate that the iteration is not
   making any progress, preventing the algorithm from continuing.")

(defmpar +powells-hybrid-unscaled+ "gsl_multiroot_fdfsolver_hybridj"
  ;; FDL
  "This algorithm is an unscaled version of *powells-hybrid*.  The steps are
   controlled by a spherical trust region |x' - x| < \delta, instead
   of a generalized region.  This can be useful if the generalized region
   estimated by *powells-hybrid* is inappropriate.")

(defmpar +newton-mfdfsolver+ "gsl_multiroot_fdfsolver_newton"
  ;; FDL
  "Newton's Method is the standard root-polishing algorithm.  The algorithm
   begins with an initial guess for the location of the solution.  On each
   iteration a linear approximation to the function F is used to
   estimate the step which will zero all the components of the residual.
   The iteration is defined by the following sequence,
   x -> x' = x - J{-1} f(x)
   where the Jacobian matrix J is computed from the derivative
   functions provided by f.  The step dx is obtained by solving
   the linear system,
   J dx = - f(x)
   using LU decomposition.")

(defmpar +gnewton-mfdfsolver+ "gsl_multiroot_fdfsolver_gnewton"
  ;; FDL
  "A modified version of Newton's method which attempts to improve
   global convergence by requiring every step to reduce the Euclidean norm
   of the residual, |f(x)|.  If the Newton step leads to an increase
   in the norm then a reduced step of relative size,
   t = (\sqrt(1 + 6 r) - 1) / (3 r)
   is proposed, with r being the ratio of norms
   |f(x')|^2/|f(x)|^2.  This procedure is repeated until a suitable step
   size is found.")

;;;;****************************************************************************
;;;; Algorithms without derivatives
;;;;****************************************************************************

(defmpar +hybrid-scaled+ "gsl_multiroot_fsolver_hybrids"
  ;; FDL
  "This is a version of the Hybrid algorithm which replaces calls to the
     Jacobian function by its finite difference approximation.  The finite
     difference approximation is computed using gsl_multiroots_fdjac
     with a relative step size of GSL_SQRT_DBL_EPSILON.")
;; Where is this function and parameter?  Only thing that shows in the
;; library is gsl_multiroot_fdjacobian.
 
(defmpar +hybrid-unscaled+ "gsl_multiroot_fsolver_hybrid"
  ;; FDL
  "A finite difference version of the Hybrid algorithm without
   internal scaling.")

(defmpar +discrete-newton+ "gsl_multiroot_fsolver_dnewton"
  ;; FDL
  "The discrete Newton algorithm is the simplest method of solving a
   multidimensional system.  It uses the Newton iteration
   x -> x - J^{-1} f(x)
   where the Jacobian matrix J is approximated by taking finite
   differences of the function f.  The approximation scheme used by
   this implementation is
   J_{ij} = (f_i(x + \delta_j) - f_i(x)) /  \delta_j
   where \delta_j is a step of size \sqrt\epsilon |x_j| with
   \epsilon being the machine precision 
   (\epsilon \approx 2.22 \times 10^-16}).
   The order of convergence of Newton's algorithm is quadratic, but the
   finite differences require n^2 function evaluations on each
   iteration.  The algorithm may become unstable if the finite differences
   are not a good approximation to the true derivatives.")

(defmpar +broyden+ "gsl_multiroot_fsolver_broyden"
  ;; FDL
  "The Broyden algorithm is a version of the discrete Newton
   algorithm which attempts to avoids the expensive update of the Jacobian
   matrix on each iteration.  The changes to the Jacobian are also
   approximated, using a rank-1 update,
   J^{-1} \to J^{-1} - (J^{-1} df - dx) dx^T J^{-1} / dx^T J^{-1} df
   where the vectors dx and df are the changes in x
   and f.  On the first iteration the inverse Jacobian is estimated
   using finite differences, as in the discrete Newton algorithm.
    
   This approximation gives a fast update but is unreliable if the changes
   are not small, and the estimate of the inverse Jacobian becomes worse as
   time passes.  The algorithm has a tendency to become unstable unless it
   starts close to the root.  The Jacobian is refreshed if this instability
   is detected (consult the source for details).

   This algorithm is included only for demonstration purposes, and is not
   recommended for serious use.")

;;;;****************************************************************************
;;;; Examples
;;;;****************************************************************************

;;; This is the example given in GSL manual, Sec. 35.8.
;;; http://www.gnu.org/software/gsl/manual/html_node/Example-programs-for-Multidimensional-Root-finding.html

;;; These examples use use scalarsp=T in the
;;; multi-dimensional-root-solver argument.  To see how vectors would
;;; be used, see minimization-multi.

(defparameter *powell-A* 1.0d4)
(defun powell (arg0 arg1)
  "Powell's test function."
  (values
   (- (* *powell-A* arg0 arg1) 1)
   (+ (exp (- arg0)) (exp (- arg1)) (- (1+ (/ *powell-A*))))))
;; not used?

(defparameter *rosenbrock-a* 1.0d0)
(defparameter *rosenbrock-b* 10.0d0)

(defun rosenbrock (arg0 arg1)
  "Rosenbrock test function."
  (values
   (* *rosenbrock-a* (- 1 arg0))
   (* *rosenbrock-b* (- arg1 (expt arg0 2)))))

(defun roots-multi-example-no-derivative
    (&optional (method +hybrid-scaled+) (print-steps t))
  "Solving Rosenbrock, the example given in Sec. 34.8 of the GSL manual."
  (let ((max-iter 1000)
	(solver (make-multi-dimensional-root-solver-f 
		 method 'rosenbrock #m(-10.0d0 -5.0d0))))
    (loop for iter from 0
       with fnval and argval
       while (and (< iter max-iter)
		  (or (zerop iter)
		      (not (multiroot-test-residual solver 1.0d-7))))
       do
       (iterate solver)
       (setf fnval (function-value solver)
	     argval (solution solver))
       (when print-steps
	 (format t "iter=~d~8tx0=~12,8g~24tx1=~12,8g~38tf0=~12,8g~52tf1=~12,8g~&"
		 iter
		 (grid:gref argval 0)
		 (grid:gref argval 1)
		 (grid:gref fnval 0)
		 (grid:gref fnval 1)))
       finally (return
		 (values (grid:gref argval 0)
			 (grid:gref argval 1)
			 (grid:gref fnval 0)
			 (grid:gref fnval 1))))))

(defun rosenbrock-df (arg0 arg1)
  "The partial derivatives of the Rosenbrock functions."
  (declare (ignore arg1))
  (values (- *rosenbrock-a*)
	  0.0d0
	  (* -2 *rosenbrock-b* arg0)
	  *rosenbrock-b*))

;;; Why is it necessary to define a function that calls the two other functions?
(defun rosenbrock-fdf (arg0 arg1)
  (multiple-value-bind (v0 v1)
      (rosenbrock arg0 arg1)
    (multiple-value-bind (j0 j1 j2 j3)
	(rosenbrock-df arg0 arg1)
      (values v0 v1 j0 j1 j2 j3))))

(defun roots-multi-example-derivative
    (&optional (method +gnewton-mfdfsolver+) (print-steps t))
  "Solving Rosenbrock with derivatives, the example given in Sec. 34.8
   of the GSL manual."
  (flet ((print-state (iter argval fnval)
	   (when print-steps
	     (format t "iter=~d~8tx0=~12,8g~24tx1=~12,8g~38tf0=~12,8g~52tf1=~12,8g~&"
		     iter
		     (grid:gref argval 0)
		     (grid:gref argval 1)
		     (grid:gref fnval 0)
		     (grid:gref fnval 1)))))
    (let ((max-iter 1000)
	  (solver (make-multi-dimensional-root-solver-fdf
		   method
		   '(rosenbrock rosenbrock-df rosenbrock-fdf)
		   #m(-10.0d0 -5.0d0))))
      (loop for iter from 0
	 with fnval = (function-value solver)
	 and argval = (solution solver)
	 while (and (< iter max-iter)
		    (not (multiroot-test-residual solver 1.0d-7)))
	 initially (print-state iter argval fnval)
	 do
	 (iterate solver)
	 (setf fnval (function-value solver)
	       argval (solution solver))
	 (print-state iter argval fnval)
	 finally (return
		   (values (grid:gref argval 0)
			   (grid:gref argval 1)
			   (grid:gref fnval 0)
			   (grid:gref fnval 1)))))))

;; To see step-by-step information as the solution progresses, make
;; the last argument T.
(save-test roots-multi
 (roots-multi-example-no-derivative +hybrid-unscaled+ nil)
 (roots-multi-example-no-derivative +hybrid-scaled+ nil)
 (roots-multi-example-no-derivative +discrete-newton+ nil)
 (roots-multi-example-no-derivative +broyden+ nil)
 (roots-multi-example-derivative +newton-mfdfsolver+ nil)
 (roots-multi-example-derivative +gnewton-mfdfsolver+ nil)
 (roots-multi-example-derivative +powells-hybrid+ nil)
 (roots-multi-example-derivative +powells-hybrid-unscaled+ nil))
