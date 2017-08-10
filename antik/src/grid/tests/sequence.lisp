;; Tests on sequences
;; Liam Healy 2014-01-05 14:20:02EST sequence.lisp
;; Time-stamp: <2014-01-05 14:32:47EST sequence.lisp>

;; Copyright 2014 Liam M. Healy
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

(lisp-unit:define-test sequences
    (lisp-unit::assert-numerical-equal
     '(5.0d0 6.0d0 7.0d0)
     (+ 2.0d0 '(3.0d0 4.0d0 5.0d0)))
  (lisp-unit::assert-numerical-equal
   '(6.0d0 8.0d0 10.0d0)
   (+ '(4.0d0 6.0d0 8.0d0) 2.0d0))
  (lisp-unit::assert-numerical-equal
   '(-1.0d0 -2.0d0 -3.0d0)
   (- 2.0d0 '(3.0d0 4.0d0 5.0d0)))
  (lisp-unit::assert-numerical-equal
   '(2.0d0 4.0d0 6.0d0)
   (- '(4.0d0 6.0d0 8.0d0) 2.0d0))
  (lisp-unit::assert-numerical-equal
   (* 2.0d0 '(3.0d0 4.0d0 5.0d0))
   '(6.0d0 8.0d0 10.0d0))
  (lisp-unit::assert-numerical-equal
   '(8.0d0 12.0d0 16.0d0)
   (* '(4.0d0 6.0d0 8.0d0) 2.0d0))
  (lisp-unit::assert-numerical-equal
   (list (cl:coerce 2/3 'double-float) 0.5d0 0.4d0)
   (/ 2.0d0 '(3.0d0 4.0d0 5.0d0)))
  (lisp-unit::assert-numerical-equal
   '(2.0d0 3.0d0 4.0d0)
   (/ '(4.0d0 6.0d0 8.0d0) 2.0d0))
  (lisp-unit::assert-numerical-equal
   '(8.0d0 8.0d0 9.0d0)
   (+ '(4.0d0 6.0d0 8.0d0) '(4.0d0 2.0d0 1.0d0)))
  (lisp-unit::assert-numerical-equal
   '(16.0d0 12.0d0 8.0d0)
   (* '(4.0d0 6.0d0 8.0d0) '(4.0d0 2.0d0 1.0d0)))
  (lisp-unit::assert-numerical-equal
   '(0.0d0 4.0d0 7.0d0)
   (- '(4.0d0 6.0d0 8.0d0) '(4.0d0 2.0d0 1.0d0)))
  (lisp-unit::assert-numerical-equal
   '(1.0d0 3.0d0 8.0d0)
   (/ '(4.0d0 6.0d0 8.0d0) '(4.0d0 2.0d0 1.0d0))))
