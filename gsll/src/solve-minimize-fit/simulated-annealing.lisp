;; Simulated Annealing
;; Liam Healy Sun Feb 11 2007 - 17:23
;; Time-stamp: <2010-06-30 19:57:28EDT simulated-annealing.lisp>
;;
;; Copyright 2007, 2008, 2009 Liam M. Healy
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

;;; /usr/include/gsl/gsl_siman.h

;;;;****************************************************************************
;;;; Simulated annealing argument structure
;;;;****************************************************************************

(fsbv:defcstruct simulated-annealing-parameters
  (n-tries :int)		; how many points to try for each step
  (iterations-fixed-T :int) ; how many iterations at each temperature?
  (step-size :double)		    ; max step size in the random walk
  ;; The following parameters are for the Boltzmann distribution
  (k :double)
  (t-initial :double)
  (mu-t :double)
  (t-min :double))

;;;;****************************************************************************
;;;; Simulated annealing states
;;;;****************************************************************************

;;; States will be saved in an array of length 4, *sa-states*, a
;;; local special.  The "state pointer" is an index of type (integer 0 3). 

(defparameter *pointer-offset* 0
  "An arbitrary offset for the generated pointers; non-negative fixnum
  value is irrelevant and changed for debugging purposes only.")

(defun state-pointer (foreign-pointer)
  (- (cffi:pointer-address foreign-pointer) *pointer-offset*))

(defun sa-state-value (foreign-pointer)
  (declare (special *sa-states*))
  (aref *sa-states* (state-pointer foreign-pointer)))

(defun make-sa-states (length)
  (declare (special *sa-states*))
  (setf *sa-states* (make-array length :adjustable t)))

(defun make-new-sa-state (&rest arguments)
  "Make a new simulated annealing state.
   Pass any arguments to user-state-maker-function."
  (declare
   (special user-state-maker-function sa-state-counter *sa-states*))
  (prog1 (cffi:make-pointer (+ *pointer-offset* sa-state-counter))
    (when (>= sa-state-counter (length *sa-states*))
      (adjust-array *sa-states* (1+ sa-state-counter)))
    (setf (aref *sa-states* sa-state-counter)
	  (apply user-state-maker-function arguments))
    (incf sa-state-counter)))

;;; The user-copy-function should take two states, and return nothing.
(defun copy-sa-state (source destination)
  (declare (special user-copy-function))
  (funcall user-copy-function
	   (sa-state-value source)
	   (sa-state-value destination)))

;;;;****************************************************************************
;;;; Callbacks
;;;;****************************************************************************

;;; The user-energy-function should take one state pointers, and
;;; return a double-float.
;;; typedef double (*gsl_siman_Efunc_t) (void *xp);
(cffi:defcallback sa-energy-function :double ((state :pointer))
  (declare (special user-energy-function))
  (funcall user-energy-function (sa-state-value state)))

;;; The user-step-function should take a rng pointer, a state pointer,
;;; and a double-float, and return nothing.  The rng pointer should be
;;; used in call to one of the distribution functions like :uniform.
;;; typedef void (*gsl_siman_step_t) (const gsl_rng *r, void *xp, double step_size);
(cffi:defcallback sa-step-function :void
    ((rng :pointer) (state :pointer) (step-size :double))
  (declare (special user-step-function))
  (funcall user-step-function rng (sa-state-value state) step-size))

;;; typedef double (*gsl_siman_metric_t) (void *xp, void *yp);
(cffi:defcallback sa-metric-function :double
    ((state1 :pointer) (state2 :pointer))
  (declare (special user-metric-function))
  (funcall user-metric-function (sa-state-value state1) (sa-state-value state2)))

;;; typedef void (*gsl_siman_copy_t) (void *source, void *dest);
(cffi:defcallback sa-copy-function :void ((source :pointer) (destination :pointer))
  (copy-sa-state source destination))

;;; typedef void * (*gsl_siman_copy_construct_t) (void *xp);
(cffi:defcallback sa-copy-constructor-function :pointer ((state :pointer))
  (let ((new (make-new-sa-state)))
    (copy-sa-state state new)
    new))

;;; Destructor?  How quaint.  Don't do anything.
;;; typedef void (*gsl_siman_destroy_t) (void *xp);
(cffi:defcallback sa-destroy-function :void ((state :pointer))
  (declare (ignore state))
  nil)

;;;;****************************************************************************
;;;; New
;;;;****************************************************************************

(export 'simulated-annealing)
(defun simulated-annealing
    (state-values
     n-tries iterations-fixed-T step-size k t-initial mu-t t-min
     generator
     state-maker-function energy-function
     step-function metric-function copy-function)
  "Perform a simulated annealing search through a given space.  The
   space is specified by providing the functions energy-function and
   metric-function.  The simulated annealing steps are generated using
   the random number generator and the function step-function.  The
   starting configuration of the system should be given by
   state-values.  The parameters n-tries, iterations-fixed-T,
   step-size, k, t-initial, mu-t, t-min control the run by providing
   the temperature schedule and other tunable parameters to the
   algorithm.  On exit the best result achieved during the search is
   returned.  If the annealing process has been successful this should
   be a good approximation to the optimal point in the space.  The
   simulated annealing routines require several user-specified
   functions to define the configuration space and energy function."
  (let ((sa-state-counter 0)
	(cl-generator)
	(user-state-maker-function state-maker-function)
	(user-energy-function energy-function)
	(user-step-function step-function)
	(user-metric-function metric-function)
	(user-copy-function copy-function))
    (declare (special
	      sa-state-counter
	      cl-generator
	      user-state-maker-function user-energy-function
	      user-step-function user-metric-function
	      user-copy-function))
    (make-sa-states 4)
    (setf cl-generator generator)
    (let ((x0-p (make-new-sa-state state-values)))
      (simulated-annealing-int
       (list n-tries iterations-fixed-T step-size k t-initial mu-t t-min)
       generator
       x0-p
       'sa-energy-function
       'sa-step-function
       'sa-metric-function)
      (sa-state-value x0-p))))

(defmfun simulated-annealing-int
    (parameters generator x0-p
		energy-function step-function metric-function)
  "gsl_siman_solve"
  (((mpointer generator) :pointer) (x0-p :pointer)
   ((cffi:get-callback energy-function) :pointer)
   ((cffi:get-callback step-function) :pointer)
   ((cffi:get-callback metric-function) :pointer)
   ((cffi:null-pointer) :pointer)	; No printing
   ((cffi:get-callback 'sa-copy-function) :pointer)
   ((cffi:get-callback 'sa-copy-constructor-function) :pointer)
   ((cffi:get-callback 'sa-destroy-function) :pointer)
   (0 sizet)
   (parameters simulated-annealing-parameters))
  :c-return :void
  :export nil
  :index simulated-annealing)

;;;;****************************************************************************
;;;; Example
;;;;****************************************************************************

;;; Trivial example, Sec. 24.3.1

(defun trivial-example-energy (state)
  (let ((x (grid:gref state 0)))
    (declare (type double-float x) (optimize (speed 3) (safety 1)))
    (* (exp (- (expt (1- x) 2))) (sin (* 8 x)))))

(defun trivial-example-step (rng-mpointer state step-size)
  (declare
   (type double-float step-size) (optimize (speed 3) (safety 1))
   ;; Ignore the RNG foreign pointer because it doesn't do us
   ;; much good, we want the CL object which we'll get from
   ;; the dynamical environment.
   (ignore rng-mpointer)
   (special cl-generator))
  (symbol-macrolet ((x (grid:gref state 0)))
    (let ((rand (sample cl-generator :uniform)))
      (declare (type double-float rand))
      (setf x (+  (the double-float x) (- (* 2.0d0 rand step-size) step-size))))))

(defun trivial-example-metric (state1 state2)
  (declare (optimize (speed 3) (safety 1)))
  (abs (- (the double-float (grid:gref state1 0))
	  (the double-float (grid:gref state2 0)))))

(defun simulated-annealing-example ()
  (simulated-annealing
   (list 15.5d0)
   ;; Parameters given in documentation
   ;; 200 10 10.0d0 1.0d0 0.002d0 1.005d0 2.0d-6 ; parameters
   ;; Parameters given in doc/examples/siman.c
   200 1000 1.0d0 1.0d0 0.008d0 1.003d0 2.0d-6 ; parameters
   (make-random-number-generator +mt19937+ 0)
   (lambda (&optional initial)
     (grid:make-foreign-array 'double-float :dimensions 1 :initial-contents initial))
   'trivial-example-energy
   'trivial-example-step
   'trivial-example-metric
   'copy))


;;; The tests used in GSL's "make check", siman/test.c

;;; exp(-square(x-1))*sin(8*x) - exp(-square(x-1000))*0.89;
(defun trivial-test-energy (state)
  (let ((x (grid:gref state 0)))
    (- (* (exp (- (expt (1- x) 2))) (sin (* 8 x)))
       (* 0.89d0 (exp (- (expt (- x 1000) 2)))))))

(defun simulated-annealing-test (initial-value)
  (simulated-annealing
   (list initial-value)
   200 1000 1.0d0 1.0d0 0.008d0 1.003d0 2.0d-6 ; parameters
   (make-random-number-generator +mt19937+ 0)
   (lambda (&optional initial)
     (grid:make-foreign-array 'double-float :dimensions 1 :initial-contents initial))
   'trivial-test-energy
   'trivial-example-step
   'trivial-example-metric
   'copy))

#|
;;; These tests should all come out within 1.0e-3 relative of the true
;;; answer
;;;  double x_min = 1.36312999455315182 ;
;;; They are kept out of the test suite for the time being because
;;; they take so long to run.
(simulated-annealing-test -10.0d0)
#<VECTOR-DOUBLE-FLOAT #(1.3631299268454313d0)>
(simulated-annealing-test 10.0d0)
#<VECTOR-DOUBLE-FLOAT #(1.3631305298767984d0)>
(simulated-annealing-test 0.6d0)
#<VECTOR-DOUBLE-FLOAT #(1.363130221515894d0)>
(simulated-annealing-test 0.5d0)
#<VECTOR-DOUBLE-FLOAT #(1.3631302551366389d0)>
(simulated-annealing-test 0.4d0)
#<VECTOR-DOUBLE-FLOAT #(1.3631296899169683d0)>
|#
