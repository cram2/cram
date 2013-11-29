;; Generic functions for optimization
;; Liam Healy 2009-01-03 12:59:07EST generic.lisp
;; Time-stamp: <2009-12-27 10:05:33EST generic.lisp>
;;
;; Copyright 2009 Liam M. Healy
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

;;;;****************************************************************************
;;;; Generic functions for solve-minimize-fit objects
;;;;****************************************************************************

(export '(iterate solution function-value last-step))

(defgeneric iterate (object)
  (:documentation "Take the next iteration step for this object."))

(defgeneric solution (object)
  (:documentation
   "The current value of the independent variable(s) that solves this object."))

(defgeneric function-value (object)
  (:documentation
   "The current value of the function that solves this object."))

(defgeneric last-step (object)
  (:documentation			; FDL
   "The last step dx taken by the solver.")) 


