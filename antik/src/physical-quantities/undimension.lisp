;; Do calculations without physical dimension.
;; Liam Healy Wed Jun 22 2005 - 22:35
;; Time-stamp: <2014-10-06 22:32:24EDT undimension.lisp>

;; Copyright 2011, 2013 Liam M. Healy
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

(in-package :antik)

(eval-when (:compile-toplevel :load-toplevel :execute)
  (export '(without-units with-units pqwu redimension dimensionless-call)))

#+example
(without-units ()
  (multiple-value-bind (mag inpunits) (pqwu (make-pq 4 'km))
    (redimension (g+ mag 5) inpunits)))
;;; #_9.000000000000000_km

#+example
(without-units ()
  (multiple-value-bind (mag inpunits)
      (pqwu (list (make-pq 4 'km/s) (make-pq 5 'km) (make-pq 6 'km)))
    (redimension (g+ mag (list 5 10 15)) inpunits)))
;;; (#_9.000000000000000_km/s #_15.000000000000000_km #_21.000000000000000_km)

(defvar *remove-units-sysunits* nil
  "System of units with which physical dimension quantities will be converted to plain numbers.")

(defmacro undimensioned-should-not-be-here (&rest args)
  "A debugging macro to help trap quantities that did not
   properly have a #'pqwu wrapping them."
  `(when *remove-units-sysunits*
    (cerror "Continue"
     "A physical dimension quantity (~a) appeared in an undimensioned calculation."
     (list ,@args))))

(defmacro without-units
    ((&optional (system-of-units '(nf-option :system-of-units))) &body body)
  "Compute everything within the dynamic scope without physical units.
   Physical dimension quantities should be wrapped with #'pqwu."
  `(let ((*remove-units-sysunits* ,system-of-units))
     ,@body))

(defmacro with-units (&body body)
  "Compute everything within the dynamic scope with physical units, 
   when there is an outer dynamic scope with without-units.
   Remove units from returned value of body."
  `(pqwu
    (let ((*remove-units-sysunits* nil))
      ,@body)))

(defun pqwu (pq)
  "If we are dynamically inside without-units, return the magnitude.
   If not, return the object as-is."
  (if *remove-units-sysunits*
      (let ((*pqval-allsame-sequence-collapse* nil))
	(with-nf-options (:system-of-units
			  (make-sysunits nil *remove-units-sysunits*))
	  (pqval pq)))
      (values pq nil nil)))

(defun redimension (magnitude unit-expression)
  "Apply units that were stripped off a quantity."
  (let ((*remove-units-sysunits* nil))
    (make-pq magnitude unit-expression T)))

;;; Generalize to multiple value returns?
(defmacro dimensionless-call (function answer-dimension &rest arguments)
  "Call a function that is incapable of handling physical dimension quantities.  This macro will consistently remove the units from all arguments, then create the answer with the specified units."
  `(without-units ()
    (redimension
     (,function
      ,@(mapcar (lambda (arg) `(with-units ,arg)) arguments))
     ,answer-dimension)))

#+example
(dimensionless-call
 lambert-battin '(expt length 3/2) (grid:norm rv1) (grid:norm rv2) (grid:norm (g- rv1 rv2)) issx)
