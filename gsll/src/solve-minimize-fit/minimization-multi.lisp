;; Multivariate minimization.
;; Liam Healy  <Tue Jan  8 2008 - 21:28>
;; Time-stamp: <2010-07-13 11:45:28EDT minimization-multi.lisp>
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

;;; /usr/include/gsl/gsl_multimin.h

;;; In the paraboloid example, I notice that the consruct 
;;; (min-test-gradient (mfdfminimizer-gradient minimizer) 1.0d-3)
;;; is constructing a CL vector-double-float (in mfdfminimizer-gradient) and
;;; then immediately pulling out the pointer (in min-test-gradient).  It
;;; is easy enough to eliminate this, but then mfdfminimizer-gradient
;;; would not be useful to a CL user.

;;; The structures fnstruct-dimension and fnstruct-dimension-fdf are, from the
;;; CFFI point of view, equally valid for gsl_multimin_function and
;;; gsl_multimin_function_fdf defined in
;;; /usr/include/gsl/gsl_multimin.h as they are for
;;; gsl_multiroot_function and gsl_multiroot_function_fdf defined in
;;; /usr/include/gsl/gsl_multiroots.h.  As far as CFFI is concerned, a
;;; pointer is a pointer, even though the C definition the functions
;;; they point to have different signatures.

;;;;****************************************************************************
;;;; Initialization
;;;;****************************************************************************

(defmobject multi-dimensional-minimizer-f
    "gsl_multimin_fminimizer"
  ((type :pointer) ((first dimensions) sizet))
  "multi-dimensional minimizer with function only"
  :documentation			; FDL
  "Make an instance of a minimizer of the given for an function of the
   given dimensions.  Optionally initialize the minimizer to minimize
   the function starting from the initial point.  The size of the
   initial trial steps is given in vector step-size. The precise
   meaning of this parameter depends on the method used."
  :callbacks
  (callback fnstruct-dimension (dimension)
	    (function :double (:input :double :foreign-array dim0) :slug))
  :initialize-suffix "set"
  :initialize-args ;; Could have one fewer argument: dimension=(dim0 initial)
  ((callback :pointer) ((mpointer initial) :pointer)
   ((mpointer step-size) :pointer))
  :singular (dimension function))

(defmobject multi-dimensional-minimizer-fdf
    "gsl_multimin_fdfminimizer"
  ((type :pointer) ((first dimensions) sizet))
  "multi-dimensional minimizer with function and derivative"
  :documentation			; FDL
  "Make an instance of a derivative-based minimizer of the given for
   an function of the given dimensions.  Optionally initialize the
   minimizer to minimize the function starting from the initial point.
   The size of the first trial step is given by step-size.  The
   accuracy of the line minimization is specified by tolernace.  The
   precise meaning of this parameter depends on the method used.
   Typically the line minimization is considered successful if the
   gradient of the function g is orthogonal to the current search
   direction p to a relative accuracy of tolerance, where dot(p,g) <
   tol |p| |g|."
  :callbacks
  (callback fnstruct-dimension-fdf (dimension)
	    (function :double (:input :double :foreign-array dim0) :slug)
	    (df :void
		(:input :double :foreign-array dim0) :slug
		(:output :double :foreign-array dim0))
	    (fdf :void
		 (:input :double :foreign-array dim0) :slug
		 (:output :double :cvector 1)
		 (:output :double :foreign-array dim0)))
  :initialize-suffix "set"
  :initialize-args
  ((callback :pointer) ((mpointer initial) :pointer)
   (step-size :double) (tolerance :double))
  :singular (dimension))

(defmfun name ((minimizer multi-dimensional-minimizer-f))
  "gsl_multimin_fminimizer_name"
  (((mpointer minimizer) :pointer))
  :definition :method
  :c-return :string
  :documentation			; FDL
  "The name of the minimizer.")

(defmfun name ((minimizer multi-dimensional-minimizer-fdf))
  "gsl_multimin_fdfminimizer_name"
  (((mpointer minimizer) :pointer))
  :definition :method
  :c-return :string
  :documentation			; FDL
  "The name of the minimizer.")

;;;;****************************************************************************
;;;; Iteration
;;;;****************************************************************************

(defmfun iterate ((minimizer multi-dimensional-minimizer-f))
  "gsl_multimin_fminimizer_iterate"
  (((mpointer minimizer) :pointer))
  :definition :method
  :callback-object minimizer
  :documentation			; FDL
  "Perform a single iteration of the minimizer.  If the iteration
   encounters an unexpected problem then an error code will be
   returned.")

(defmfun iterate ((minimizer multi-dimensional-minimizer-fdf))
  "gsl_multimin_fdfminimizer_iterate"
  (((mpointer minimizer) :pointer))
  :definition :method
  :callback-object minimizer
  :documentation			; FDL
  "Perform a single iteration of the minimizer.  If the iteration
   encounters an unexpected problem then an error code will be
   returned.")

(defmfun solution ((minimizer multi-dimensional-minimizer-f))
  "gsl_multimin_fminimizer_x"
  (((mpointer minimizer) :pointer))
  :definition :method
  :callback-object minimizer
  :c-return (crtn :pointer)
  :return ((make-foreign-array-from-mpointer crtn))
  :documentation			; FDL
  "The current best estimate of the location of the minimum.")

(defmfun solution ((minimizer multi-dimensional-minimizer-fdf))
  "gsl_multimin_fdfminimizer_x"
  (((mpointer minimizer) :pointer))
  :definition :method
  :callback-object minimizer
  :c-return (crtn :pointer)
  :return ((make-foreign-array-from-mpointer crtn))
  :documentation			; FDL
  "The current best estimate of the location of the minimum.")

(defmfun function-value ((minimizer multi-dimensional-minimizer-f))
  "gsl_multimin_fminimizer_minimum"
  (((mpointer minimizer) :pointer))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "The current best estimate of the value of the minimum.")

(defmfun function-value ((minimizer multi-dimensional-minimizer-fdf))
  "gsl_multimin_fdfminimizer_minimum"
  (((mpointer minimizer) :pointer))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "The current best estimate of the value of the minimum.")

(defmfun size ((minimizer multi-dimensional-minimizer-f))
  "gsl_multimin_fminimizer_size"
  (((mpointer minimizer) :pointer))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A minimizer-specific characteristic size for the minimizer.")

(defmfun mfdfminimizer-gradient (minimizer)
  "gsl_multimin_fdfminimizer_gradient"
  (((mpointer minimizer) :pointer))
  :c-return (crtn :pointer)
  :return ((make-foreign-array-from-mpointer crtn))
  :documentation			; FDL
  "The current best estimate of the gradient for the minimizer.")

(defmfun mfdfminimizer-restart (minimizer)
  "gsl_multimin_fdfminimizer_restart"
  (((mpointer minimizer) :pointer))
  :documentation			; FDL
  "Reset the minimizer to use the current point as a
   new starting point.")

;;;;****************************************************************************
;;;; Stopping criteria
;;;;****************************************************************************

(defmfun min-test-gradient (gradient absolute-error)
  "gsl_multimin_test_gradient"
  (((mpointer gradient) :pointer) (absolute-error :double))
  :inputs (gradient)
  :c-return :success-continue
  :documentation			; FDL
  "Test the norm of the gradient against the
   absolute tolerance absolute-error.  The gradient of a multidimensional
   function goes to zero at a minimum. The test returns T
   if |g| < epsabs is achieved, and NIL otherwise.  A suitable choice of
   absolute-error can be made from the desired accuracy in the function for
   small variations in x.  The relationship between these quantities
   \delta f = g \delta x.")

(defmfun min-test-size (size absolute-error)
  "gsl_multimin_test_size"
  ((size :double) (absolute-error :double))
  :c-return :success-continue
  :documentation			; FDL
  "Test the minimizer specific characteristic size (if applicable to
   the used minimizer) against absolute tolerance absolute-error.  The test
   returns T if the size is smaller than tolerance, and NIL otherwise.")

;;;;****************************************************************************
;;;; Algorithms
;;;;****************************************************************************

(defmpar +conjugate-fletcher-reeves+
    "gsl_multimin_fdfminimizer_conjugate_fr"
  ;; FDL
  "The Fletcher-Reeves conjugate gradient algorithm. The conjugate
   gradient algorithm proceeds as a succession of line minimizations. The
   sequence of search directions is used to build up an approximation to the
   curvature of the function in the neighborhood of the minimum.  

   An initial search direction p is chosen using the gradient, and
   line minimization is carried out in that direction.  The accuracy of
   the line minimization is specified by the parameter tol.  The minimum
   along this line occurs when the function gradient g and the search
   direction p are orthogonal.  The line minimization terminates when
   dot(p,g) < tol |p| |g|.  The search direction is updated using the
   Fletcher-Reeves formula p' = g' - \beta g where \beta=-|g'|^2/|g|^2,
   and the line minimization is then repeated for the new search
   direction.")

(defmpar +conjugate-polak-ribiere+
    "gsl_multimin_fdfminimizer_conjugate_pr"
  ;; FDL
  "The Polak-Ribiere conjugate gradient algorithm.  It is similar to
   the Fletcher-Reeves method, differing only in the choice of the
   coefficient \beta. Both methods work well when the evaluation point is
   close enough to the minimum of the objective function that it is well
   approximated by a quadratic hypersurface.")

(defmpar +vector-bfgs+
    "gsl_multimin_fdfminimizer_vector_bfgs"
  ;; FDL
  "The vector Broyden-Fletcher-Goldfarb-Shanno (BFGS) conjugate
   gradient algorithm.  It is a quasi-Newton method which builds up an
   approximation to the second derivatives of the function using the
   difference between successive gradient vectors.  By combining the
   first and second derivatives the algorithm is able to take Newton-type
   steps towards the function minimum, assuming quadratic behavior in
   that region.")

(defmpar +vector-bfgs2+
    "gsl_multimin_fdfminimizer_vector_bfgs2"
  ;; FDL
  "The vector Broyden-Fletcher-Goldfarb-Shanno (BFGS) conjugate
   gradient algorithm.  It is a quasi-Newton method which builds up an
   approximation to the second derivatives of the function using the
   difference between successive gradient vectors.  By combining the
   first and second derivatives the algorithm is able to take Newton-type
   steps towards the function minimum, assuming quadratic behavior in
   that region.

   This version is the most efficient version available, and is a
   faithful implementation of the line minimization scheme described
   in Fletcher's Practical Methods of Optimization, Algorithms 2.6.2
   and 2.6.4. It supercedes the original bfgs routine and requires
   substantially fewer function and gradient evaluations. The
   user-supplied tolerance tol corresponds to the parameter \sigma
   used by Fletcher. A value of 0.1 is recommended for typical
   use (larger values correspond to less accurate line searches)."
  :gsl-version (1 9))

(defmpar +simplex-nelder-mead-on2+
    "gsl_multimin_fminimizer_nmsimplex"
  ;; FDL
  "The Simplex algorithm of Nelder and Mead. It constructs 
   n vectors p_i from the
   starting vector initial and the vector step-size as follows:
   p_0 = (x_0, x_1, ... , x_n) 
   p_1 = (x_0 + step_size_0, x_1, ... , x_n) 
   p_2 = (x_0, x_1 + step_size_1, ... , x_n) 
   ... = ...
   p_n = (x_0, x_1, ... , x_n+step_size_n)
   These vectors form the n+1 vertices of a simplex in n
   dimensions.  On each iteration the algorithm tries to improve
   the parameter vector p_i corresponding to the highest
   function value by simple geometrical transformations.  These
   are reflection, reflection followed by expansion, contraction and multiple
   contraction. Using these transformations the simplex moves through 
   the parameter space towards the minimum, where it contracts itself.  

   After each iteration, the best vertex is returned.  Note, that due to
   the nature of the algorithm not every step improves the current
   best parameter vector.  Usually several iterations are required.

   The routine calculates the minimizer specific characteristic size as the
   average distance from the geometrical center of the simplex to all its
   vertices.  This size can be used as a stopping criteria, as the simplex
   contracts itself near the minimum. The size is returned by the function
   #'mfminimizer-size.

   This version is O(n^2).")

(defmpar +simplex-nelder-mead+
    "gsl_multimin_fminimizer_nmsimplex2"
  ;; FDL
  "The Simplex algorithm of Nelder and Mead. It constructs 
   n vectors p_i from the
   starting vector initial and the vector step-size as follows:
   p_0 = (x_0, x_1, ... , x_n) 
   p_1 = (x_0 + step_size_0, x_1, ... , x_n) 
   p_2 = (x_0, x_1 + step_size_1, ... , x_n) 
   ... = ...
   p_n = (x_0, x_1, ... , x_n+step_size_n)
   These vectors form the n+1 vertices of a simplex in n
   dimensions.  On each iteration the algorithm tries to improve
   the parameter vector p_i corresponding to the highest
   function value by simple geometrical transformations.  These
   are reflection, reflection followed by expansion, contraction and multiple
   contraction. Using these transformations the simplex moves through 
   the parameter space towards the minimum, where it contracts itself.  

   After each iteration, the best vertex is returned.  Note, that due to
   the nature of the algorithm not every step improves the current
   best parameter vector.  Usually several iterations are required.

   The routine calculates the minimizer specific characteristic size as the
   average distance from the geometrical center of the simplex to all its
   vertices.  This size can be used as a stopping criteria, as the simplex
   contracts itself near the minimum. The size is returned by the function
   #'mfminimizer-size.

   This version is O(n); calculates the size of simplex as the rms
   distance of each vertex from the center rather than the mean
   distance, which has the advantage of allowing a linear update."
  :gsl-version (1 12))

;;;;****************************************************************************
;;;; Examples
;;;;****************************************************************************

;;; Examples from Sec. 35.8.

(defparameter *paraboloid-center* #(1.0d0 2.0d0))

;;; Example without derivatives taking scalars.

(defun paraboloid-scalar (x y)
  "A paraboloid function of two arguments, given in GSL manual Sec. 35.4.
   This version takes scalar arguments."
  (let ((dp0 (aref *paraboloid-center* 0))
	(dp1 (aref *paraboloid-center* 1)))
    (+ (* 10 (expt (- x dp0) 2))
       (* 20 (expt (- y dp1) 2))
       30)))

(defun multimin-example-no-derivative
    (&optional (method +simplex-nelder-mead-on2+) (print-steps t))
  (let ((step-size (grid:make-foreign-array 'double-float :dimensions 2)))
    (set-all step-size 1.0d0)
    (let ((minimizer
	   (make-multi-dimensional-minimizer-f
	    method 2 'paraboloid-scalar
	    #m(5.0d0 7.0d0) step-size)))
      (loop with status = T and size
	 for iter from 0 below 100
	 while status
	 do (iterate minimizer)
	 (setf size
	       (size minimizer)
	       status
	       (not (min-test-size size 1.0d-2)))
	 (when print-steps
	   (let ((x (solution minimizer)))
	     (format t "~d~6t~10,6f~18t~10,6f~28t~12,9f~40t~8,3f~&"
		     iter (grid:gref x 0) (grid:gref x 1)
		     (function-value minimizer)
		     size)))
	 finally
	 (return
	   (let ((x (solution minimizer)))
	     (values (grid:gref x 0) (grid:gref x 1) (function-value minimizer))))))))

;;; Example using derivatives, taking a vector argument.
;;; Note that these functions are written to read mpointers to
;;; vector-double-float.  They could as well have been written to
;;; accept the correct number of scalar double-floats.

(defun paraboloid-vector (xy)
  "A paraboloid function of two arguments, given in GSL manual Sec. 35.4.
   This version takes a vector-double-float argument."
  ;; An alternative access to the passed-in vector would be to
  ;; bind a variable to
  ;; (make-foreign-array-from-mpointer mpointer 'double-float :vector)
  ;; and then call grid:gref on it.
  (let ((x (grid:gref xy 0))
	(y (grid:gref xy 1))
	(dp0 (aref *paraboloid-center* 0))
	(dp1 (aref *paraboloid-center* 1)))
    (+ (* 10 (expt (- x dp0) 2))
       (* 20 (expt (- y dp1) 2))
       30)))

(defun paraboloid-derivative (xy output)
  (let ((x (grid:gref xy 0))
	(y (grid:gref xy 1))
	(dp0 (aref *paraboloid-center* 0))
	(dp1 (aref *paraboloid-center* 1)))
    (setf (grid:gref output 0)
	  (* 20 (- x dp0))
	  (grid:gref output 1)
	  (* 40 (- y dp1)))))

(defun paraboloid-and-derivative
    (arguments-gv-pointer value-pointer derivative-gv-pointer)
  (prog1
      (setf (grid:gref value-pointer 0)
	    (paraboloid-vector arguments-gv-pointer))
    (paraboloid-derivative
     arguments-gv-pointer derivative-gv-pointer)))

(defun multimin-example-derivative
    (&optional (method +conjugate-fletcher-reeves+) (print-steps t))
  "This is an example solving the multidimensional minimization problem
   of a paraboloid using the derivative.  The callback functions
   paraboloid-vector and paraboloid-derivative expect vectors.
   Contrast this with multimin-example-derivative-scalars, which
   expects and returns the scalar components."
  (let* ((initial #m(5.0d0 7.0d0))
	 (minimizer
	  (make-multi-dimensional-minimizer-fdf
	   method 2
	   '(paraboloid-vector paraboloid-derivative paraboloid-and-derivative)
	   initial 0.01d0 1.0d-4 nil)))
    (loop with status = T
       for iter from 0 below 100
       while status
       do
       (iterate minimizer)
       (setf status
	     (not (min-test-gradient
		   (mfdfminimizer-gradient minimizer)
		   1.0d-3)))
       (when print-steps
	 (let ((x (solution minimizer)))
	   (format t "~d~6t~10,6f~18t~10,6f~28t~12,9f~&"
		   iter (grid:gref x 0) (grid:gref x 1)
		   (function-value minimizer))))
       finally
       (return
	 (let ((x (solution minimizer)))
	   (values (grid:gref x 0) (grid:gref x 1) (function-value minimizer)))))))

(defun paraboloid-derivative-scalar (x y)
  (let ((dp0 (aref *paraboloid-center* 0))
	(dp1 (aref *paraboloid-center* 1)))
    (values
     (* 20 (- x dp0))
     (* 40 (- y dp1)))))

(defun paraboloid-and-derivative-scalar (x y)
  (values-list
   (cons (paraboloid-scalar x y)
	 (multiple-value-list (paraboloid-derivative-scalar x y)))))

(defun multimin-example-derivative-scalars
    (&optional (method +conjugate-fletcher-reeves+) (print-steps t))
  "This is an example solving the multidimensional minimization problem
   of a paraboloid using the derivative.  The callback functions
   paraboloid-scalar and paraboloid-derivative-scalar expect scalars.
   Contrast this with multimin-example-derivative, which
   expects and returns vectors."
  (let* ((initial #m(5.0d0 7.0d0))
	 (minimizer
	  (make-multi-dimensional-minimizer-fdf
	   method 2
	   '(paraboloid-scalar paraboloid-derivative-scalar paraboloid-and-derivative-scalar)
	   initial 0.01d0 1.0d-4 t)))
    (loop with status = T
       for iter from 0 below 100
       while status
       do
       (iterate minimizer)
       (setf status
	     (not (min-test-gradient
		   (mfdfminimizer-gradient minimizer)
		   1.0d-3)))
       (when print-steps
	 (let ((x (solution minimizer)))
	   (format t "~d~6t~10,6f~18t~10,6f~28t~12,9f~&"
		   iter (grid:gref x 0) (grid:gref x 1)
		   (function-value minimizer))))
       finally
       (return
	 (let ((x (solution minimizer)))
	   (values (grid:gref x 0) (grid:gref x 1) (function-value minimizer)))))))

(save-test minimization-multi
 (multimin-example-no-derivative +simplex-nelder-mead-on2+ nil)
 (multimin-example-derivative +conjugate-fletcher-reeves+ nil)
 (multimin-example-derivative +conjugate-polak-ribiere+ nil)
 (multimin-example-derivative +vector-bfgs+ nil)
 (multimin-example-derivative +vector-bfgs2+ nil)
 (multimin-example-derivative-scalars +vector-bfgs2+ nil))
