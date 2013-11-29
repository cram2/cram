;; ODE system setup
;; Liam Healy, Sun Apr 15 2007 - 14:19
;; Time-stamp: <2010-06-30 19:57:28EDT ode-system.lisp>
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

(export '(with-ode-integration))

(defmacro with-ode-integration
    ((function time step-size max-time dependent dimensions
	       &key jacobian (scalarsp t) (stepper '+step-rk8pd+)
	       (absolute-error 1.0d-6) (relative-error 0.0d0))
     &body body)
  "Environment for integration of ordinary differential equations."
  ;; Note: the case jacobian=nil is not properly handled yet; it
  ;; should put a null pointer into the struct that is passed to GSL.
  (let ((dep (make-symbol "DEP"))
	(ctime (make-symbol "CTIME"))
	(cstep (make-symbol "CSTEP")))
    `(let ((stepperobj
	    (make-ode-stepper
	     ,stepper ,dimensions ',function ',jacobian ,scalarsp))
	   (control (make-y-control ,absolute-error ,relative-error))
	   (evolve (make-ode-evolution ,dimensions))
	   (,dep
	    (grid:make-foreign-array 'double-float :dimensions ,dimensions))
	   (,ctime (grid:make-foreign-array 'double-float :dimensions 1))
	   (,cstep (grid:make-foreign-array 'double-float :dimensions 1)))
       (symbol-macrolet
	   ((,time (grid:gref ,ctime 0))
	    (,step-size (grid:gref ,cstep 0))
	    ,@(loop for symb in dependent
		 for i from 0
		 collect `(,symb (grid:gref ,dep ,i))))
	 (flet ((next-step ()
		  (apply-evolution
		   evolve ,ctime ,dep ,cstep control stepperobj ,max-time)))
	   ,@body)))))
