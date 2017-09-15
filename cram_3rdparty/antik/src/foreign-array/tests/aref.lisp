;; Test grids
;; Liam Healy 2011-10-23 17:46:07EDT aref.lisp
;; Time-stamp: <2011-10-23 18:16:56EDT aref.lisp>
;;
;; Copyright 2011 Liam M. Healy
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

(defmacro make-grid-set-get (element-type grid-type value)
  `(let ((grid (make-simple-grid :element-type ',element-type :grid-type ',grid-type)))
    (setf (aref grid 1) ,value)
    (aref grid 1)))

;;; Single element set and get with grid:aref
(lisp-unit:define-test aref
    (lisp-unit:assert-numerical-equal
     5
     (make-grid-set-get (signed-byte 32) array 5))
  (lisp-unit:assert-numerical-equal
   5
   (make-grid-set-get (signed-byte 32) foreign-array 5))
  (lisp-unit:assert-numerical-equal
   5
   (make-grid-set-get (signed-byte 64) array 5))
  (lisp-unit:assert-numerical-equal
   5
   (make-grid-set-get (signed-byte 64) foreign-array 5))
  (lisp-unit:assert-numerical-equal
   5.5f0
   (make-grid-set-get single-float array 5.5f0))
  (lisp-unit:assert-numerical-equal
   5.5f0
   (make-grid-set-get single-float foreign-array 5.5f0))
  (lisp-unit:assert-numerical-equal
   5.5d0
   (make-grid-set-get double-float array 5.5d0))
  (lisp-unit:assert-numerical-equal
   5.5d0
   (make-grid-set-get double-float foreign-array 5.5d0))
  (lisp-unit:assert-numerical-equal
   #C(2.5f0 3.5f0)
   (make-grid-set-get (complex single-float) array #C(2.5f0 3.5f0)))
  (lisp-unit:assert-numerical-equal
   #C(2.5f0 3.5f0)
   (make-grid-set-get (complex single-float) foreign-array #C(2.5f0 3.5f0)))
  (lisp-unit:assert-numerical-equal
   #C(2.5d0 3.5d0)
   (make-grid-set-get (complex double-float) array #C(2.5d0 3.5d0)))
  (lisp-unit:assert-numerical-equal
   #C(2.5d0 3.5d0)
   (make-grid-set-get (complex double-float) foreign-array #C(2.5d0 3.5d0))))
