;; Tests
;; Liam Healy 2009-12-28 22:33:55EST higher.lisp
;; Time-stamp: <2009-12-28 22:53:40EST higher.lisp>
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

(in-package :grid)

(lisp-unit:define-test higher
  (lisp-unit:assert-numerical-equal
   #(0.0d0 0.0d0 1.0d0)
   (cross #(1.0d0 0.0d0 0.0d0) #(0.0 1.0d0 0.0d0)))
  (lisp-unit:assert-numerical-equal
   #(1.0d0 0.0d0 0.0d0)
   (cross #(0.0 1.0d0 0.0d0) #(0.0d0 0.0d0 1.0d0)))
  (lisp-unit:assert-numerical-equal
   #(0.0 1.0d0 0.0d0)
   (cross #(0.0d0 0.0d0 1.0d0) #(1.0d0 0.0d0 0.0d0)))
  (lisp-unit:assert-numerical-equal
   7.0710678118654755d0
   (euclidean #(3.0d0 4.0d0 5.0d0))))
