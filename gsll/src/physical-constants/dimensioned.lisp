;; Dimensioned physical constants.
;; Liam Healy 2013-02-24 00:28:51EST dimensioned.lisp
;; Time-stamp: <2013-02-24 00:45:33EST dimensioned.lisp>
;;
;; Copyright 2013 Liam M. Healy
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

(export '(speedoflight gravitationalconstant planckh planckhbar))

(define-physical-constant speedoflight (make-pq +mksa-speed-of-light+ "m/s"))
(define-physical-constant gravitationalconstant (make-pq +mksa-gravitational-constant+ "m^3/kg-s^2"))
(define-physical-constant planckh (make-pq +mksa-plancks-constant-h+ "m^2-kg/s"))
(define-physical-constant planckhbar (make-pq +mksa-plancks-constant-hbar+ "m^2-kg/s"))
