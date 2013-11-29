;; Univariate minimization
;; Liam Healy Tue Jan  8 2008 - 21:02
;; Time-stamp: <2010-07-01 11:55:38EDT minimization-one.lisp>
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

;;; /usr/include/gsl/gsl_min.h

;;;;****************************************************************************
;;;; Initialization
;;;;****************************************************************************

;; The one-dimensional minimizer uses a function (no derivative) only.
(defmobject one-dimensional-minimizer
    "gsl_min_fminimizer"
  ((type :pointer))
  "one-dimensional minimizer"
  :documentation
  "Make an instance of a minimizer of the given type.  Specify
   a guess of the minimum point, the search interval
   (x-minimum x-lower x-upper) and optionally 
   function values at those points (f-minimum f-lower f-upper)."
  :callbacks
  (callback fnstruct nil (function :double (:input :double) :slug))
  :initialize-suffix ("set" "set_with_values")
  :initialize-args
  (((callback :pointer) (x-minimum :double)
    (x-lower :double) (x-upper :double))
   ((callback :pointer)
    (x-minimum :double) (f-minimum :double)
    (x-lower :double) (f-lower :double)
    (x-upper :double) (f-upper :double)))
  :switch (f-minimum f-lower f-upper)
  :singular (function)
  :arglists-function
  (lambda (set)
    `((type &optional (function nil ,set)
	    x-minimum x-lower x-upper f-minimum f-lower f-upper)
      (:type type)
      (:functions (list function) :x-minimum
		  x-minimum :f-minimum f-minimum :x-lower x-lower
		  :f-lower f-lower :x-upper x-upper :f-upper
		  f-upper))))

(defmfun name ((minimizer one-dimensional-minimizer))
  "gsl_min_fminimizer_name"
  (((mpointer minimizer) :pointer))
  :definition :method
  :c-return :string
  :documentation			; FDL
  "The name of the minimizer.")

;;;;****************************************************************************
;;;; Iteration
;;;;****************************************************************************

(defmfun iterate ((minimizer one-dimensional-minimizer))
  "gsl_min_fminimizer_iterate"
  (((mpointer minimizer) :pointer))
  :definition :method
  :c-return :success-continue
  :callback-object minimizer
  :documentation			; FDL
  "Perform a single iteration of the minimizer.  The following
   errors may be signalled: 'bad-function-supplied,
   the iteration encountered a singular point where the function or its
   derivative evaluated to infinity or NaN, or
   :FAILURE, the algorithm could not improve the current best approximation or
   bounding interval.")

(defmfun solution ((minimizer one-dimensional-minimizer))
  "gsl_min_fminimizer_x_minimum"
  (((mpointer minimizer) :pointer))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "The current estimate of the position of the minimum for the minimizer.")

(defmfun fminimizer-x-lower (minimizer)
  "gsl_min_fminimizer_x_lower"
  (((mpointer minimizer) :pointer))
  :c-return :double
  :documentation			; FDL
  "The current lower bound of the interval for the minimizer.")

(defmfun fminimizer-x-upper (minimizer)
  "gsl_min_fminimizer_x_upper"
  (((mpointer minimizer) :pointer))
  :c-return :double
  :documentation			; FDL
  "The current upper bound of the interval for the minimizer.")

(defmfun function-value ((minimizer one-dimensional-minimizer))
  "gsl_min_fminimizer_f_minimum"
  (((mpointer minimizer) :pointer))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "The value of the function at the current estimate of the minimum for the
   minimizer.")

(defmfun fminimizer-f-lower (minimizer)
  "gsl_min_fminimizer_f_lower"
  (((mpointer minimizer) :pointer))
  :c-return :double
  :documentation			; FDL
  "The value of the function at the current estimate of the lower bound
   for the minimizer.")

(defmfun fminimizer-f-upper (minimizer)
  "gsl_min_fminimizer_f_upper"
  (((mpointer minimizer) :pointer))
  :c-return :double
  :documentation			; FDL
  "The value of the function at the current estimate of the upper bound
   for the minimizer.")

;;;;****************************************************************************
;;;; Stopping parameters
;;;;****************************************************************************

(defmfun min-test-interval (lower upper absolute-error relative-error)
  "gsl_min_test_interval"
  ((lower :double) (upper :double)
   (absolute-error :double) (relative-error :double))
  :c-return :success-continue		; guess that this is s-c, not s-f
  :documentation			; FDL
  "Test for the convergence of the interval [lower,upper]
   with absolute error and relative error specified.
   The test returns T if the following condition is achieved:
   |a - b| < epsabs + epsrel min(|a|,|b|) 
   when the interval x = [a,b] does not include the origin.  If the
   interval includes the origin then min(|a|,|b|) is replaced by
   zero (which is the minimum value of |x| over the interval).  This
   ensures that the relative error is accurately estimated for minima close
   to the origin.

   This condition on the interval also implies that any estimate of the
   minimum x_m in the interval satisfies the same condition with respect
   to the true minimum x_m^*,
   |x_m - x_m^*| < epsabs + epsrel x_m^*
   assuming that the true minimum x_m^* is contained within the interval.")

;;;;****************************************************************************
;;;; Minimization algorithms
;;;;****************************************************************************

(defmpar +golden-section-fminimizer+ "gsl_min_fminimizer_goldensection"
  "The golden section algorithm is the simplest method of bracketing
   the minimum of a function.  It is the slowest algorithm provided by the
   library, with linear convergence.

   On each iteration, the algorithm first compares the subintervals from
   the endpoints to the current minimum.  The larger subinterval is divided
   in a golden section (using the famous ratio (3-sqrt 5)/2 =
   0.3189660...) and the value of the function at this new point is
   calculated.  The new value is used with the constraint f(a') >
   f(x') < f(b') to a select new interval containing the minimum, by
   discarding the least useful point.  This procedure can be continued
   indefinitely until the interval is sufficiently small.  Choosing the
   golden section as the bisection ratio can be shown to provide the
   fastest convergence for this type of algorithm.")

(defmpar +brent-fminimizer+ "gsl_min_fminimizer_brent"
  "The Brent minimization algorithm combines a parabolic
   interpolation with the golden section algorithm.  This produces a fast
   algorithm which is still robust.

   The outline of the algorithm can be summarized as follows: on each
   iteration Brent's method approximates the function using an
   interpolating parabola through three existing points.  The minimum of the
   parabola is taken as a guess for the minimum.  If it lies within the
   bounds of the current interval then the interpolating point is accepted,
   and used to generate a smaller interval.  If the interpolating point is
   not accepted then the algorithm falls back to an ordinary golden section
   step.  The full details of Brent's method include some additional checks
   to improve convergence.")

;;;;****************************************************************************
;;;; Example
;;;;****************************************************************************

;;; This is the example given in Sec. 33.8.  The results are different
;;; than given there.

(defun minimization-one-example
    (&optional (minimizer-type +brent-fminimizer+) 
               (print-steps t) 
               (with-values t))
  "Solving a minimum, the example given in Sec. 33.8 of the GSL manual."
  (flet ((f (x) (1+ (cos x))))
   (let ((max-iter 100)
         (minimizer
           (if with-values
             (make-one-dimensional-minimizer 
               minimizer-type #'f 
               2.0d0 0.0d0 6.0d0 
               (f 2.0d0) (f 0.0d0) (f 6.0d0))
             (make-one-dimensional-minimizer
               minimizer-type #'f 2.0d0 0.0d0 6.0d0))))
     (when print-steps
       (format
        t
        "iter ~6t   [lower ~24tupper] ~36tmin ~44tmin err ~54tupper-lower~&"))
     (loop for iter from 0
        for min = (solution minimizer)
        for lower = (fminimizer-x-lower minimizer)
        for upper = (fminimizer-x-upper minimizer)
        do (iterate minimizer)
        (when print-steps
          (format t "~d~6t~10,6f~18t~10,6f~28t~12,9f ~44t~10,4g ~10,4g~&"
                  iter lower upper
                  min (- min pi)
                  (- upper lower)))

        while  (and (< iter max-iter)
                    ;; abs and rel error swapped in example?
                    (not (min-test-interval lower upper 0.001d0 0.0d0)))
        finally
        (return (values iter lower upper min (- min pi) (- upper lower)))))))

(save-test minimization-one
 (minimization-one-example +brent-fminimizer+ nil nil)
 (minimization-one-example +golden-section-fminimizer+ nil nil)
 (minimization-one-example +brent-fminimizer+ nil t)
 (minimization-one-example +golden-section-fminimizer+ nil t))
