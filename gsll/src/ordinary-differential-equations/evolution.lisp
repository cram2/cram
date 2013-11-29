;; Evolution functions for ODE integration.
;; Liam Healy, Sun Sep 30 2007 - 14:31
;; Time-stamp: <2010-06-27 18:13:51EDT evolution.lisp>
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

(defmobject ode-evolution "gsl_odeiv_evolve"
  ((dimensions sizet))
  "evolution for ordinary differential equations"
  :documentation "Make an object to advance the ODE solution."
  :initialize-suffix "reset"
  :initialize-args nil)

(defmfun apply-evolution
    (evolution time y step-size control stepper max-time)
  "gsl_odeiv_evolve_apply"
  (((mpointer evolution) :pointer) ((mpointer control) :pointer)
   ((mpointer stepper) :pointer)
   ((callback-struct stepper) :pointer) ((foreign-pointer time) :pointer)
   (max-time :double)
   ((foreign-pointer step-size) :pointer) ((foreign-pointer y) :pointer))
  :inputs (time step-size y)
  :outputs (time step-size y)
  :callback-object ((stepper ode-stepper))
  :documentation			; FDL
  "Advance the system (e, dydt) from time
   and position y using the stepping function step.
   The new time and position are stored in time and y on output.
   The initial step-size supplied, but this will be modified
   using the control function to achieve the appropriate error
   bound if necessary.  The routine may make several calls to step in
   order to determine the optimum step-size. If the step-size has been
   changed the value of step-size will be modified on output.  The maximum
   time max-time is guaranteed not to be exceeded by the time-step.  On the
   final time-step the value of time will be set to t1 exactly.")
