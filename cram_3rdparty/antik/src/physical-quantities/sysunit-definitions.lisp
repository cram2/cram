;; Definitions of systems of units
;; Liam Healy Tue Mar 19 2002 - 18:07
;; Time-stamp: <2013-06-02 23:12:38EDT sysunit-definitions.lisp>

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

;;; The file unit-definitions must be compiled and loaded before this can
;;; be compiled.

;;;  "Systeme internationale system of units."
(define-system-of-units si
    (meter second kilogram kelvin
	   ampere mole candela radian
	   newton watt joule pascal coulomb volt
	   farad ohm siemens tesla weber henry))

;;; "The centimeter-gram-seconds system of units."
(define-system-of-units cgs
    (centimeter radian gram second dyne watt erg dynes-per-square-centimeter))

;;; "The English system of units."
(define-system-of-units english
    (foot radian slug second pound-force
	  horsepower foot-pound pounds-per-square-inch))

;;; "A pseudo-system of units with dimension names for formatting units; it cannot be used for conversions."
(setf (find-sysunits 'basis-physical-dimensions)
      (make-sysunits *basis-dimensions*))

(set-system-of-units 'si)
