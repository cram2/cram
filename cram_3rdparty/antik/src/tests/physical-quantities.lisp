;; Tests of physical quantities
;; Liam Healy 2011-01-09 17:38:41EST tests.lisp
;; Time-stamp: <2015-04-13 22:59:07EDT physical-quantities.lisp>

;; Copyright 2011, 2013, 2015 Liam M. Healy
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
(named-readtables:in-readtable :antik)

(defmethod lisp-unit:numerical-equal
    ((result1 physical-quantity) (result2 physical-quantity) &rest keys &key test)
  (declare (ignorable test))
  (multiple-value-bind (mag2 units2)
      (pqval result2)
    (and
     (apply 'lisp-unit::numerical-equal (pq-magnitude result1) mag2 keys)
     (check-dimension result1 units2 nil))))

;;; Radians count as a number
(defmethod lisp-unit:numerical-equal
    ((result1 physical-quantity) (result2 number) &rest keys &key test)
  (declare (ignorable test))
  (and (check-dimension result1 'angle nil)
       (apply 'lisp-unit::numerical-equal (pq-magnitude result1) result2 keys)))

(defmethod lisp-unit:numerical-equal
    ((result1 number) (result2 physical-quantity) &rest keys &key test)
  (declare (ignorable test))
  (and (check-dimension result2 'angle nil)
       (apply 'lisp-unit::numerical-equal result1 (pq-magnitude result2) keys)))

(defun check-pq (object value units)
  "Check that the object is a physical-quantity and that its magnitude
   and units are as given."
  (and (physical-quantity-p object)
       (lisp-unit::numerical-equal value (pq-magnitude object))
       (check-dimension object units nil)))

(lisp-unit:define-test units
  (lisp-unit:assert-true (check-pq #_1_m 1.0d0 'm))
  (lisp-unit:assert-false (check-pq #_1_m 1.0d0 's))
  (lisp-unit:assert-numerical-equal #_2.0_m (antik:+ #_1_m #_1_m))
  (lisp-unit:assert-error 'simple-error (antik:+ #_1_m #_1_s))
  (lisp-unit:assert-numerical-equal #_0.5_m/s (/ #_1_m #_2_s))
  (lisp-unit:assert-numerical-equal #_6.0_m^2 (* #_3_m #_2_m))
  (lisp-unit:assert-numerical-equal #_0.3048_m #_1.0_foot)
  (lisp-unit:assert-numerical-equal #_0.45359237_kg #_1.0_pound)
  (lisp-unit:assert-numerical-equal #_3600_s #_1_hour))

(lisp-unit:define-test system-of-units
    (lisp-unit::assert-numerical-equal
     10.0d0
     (with-system-of-units (t degree) (pqval #_10.0d0_degrees)))
  (lisp-unit::assert-numerical-equal
   0.3048d0
   (antik::pq-magnitude
    (with-system-of-units (english) (make-pq 1.0d0 'length))))
  (lisp-unit::assert-numerical-equal
   100000.0d0
   (with-system-of-units (cgs)
     (pqval #_1_km))))
