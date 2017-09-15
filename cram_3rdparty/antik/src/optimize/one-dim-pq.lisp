;; Minimize or maximize the function in one variable, method with physical dimension
;; Liam Healy 2013-11-30 11:59:12EST one-dim-pq.lisp
;; Time-stamp: <2013-11-30 12:02:34EST one-dim-pq.lisp>

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

;;; This requires GSLL and physical-dimension

(in-package :antik)

(defmethod root-1d (function (initial physical-quantity) derivative
		    &optional
		      (absolute-error 0.0001d0)
		      (relative-error 0.0001d0)
		      (method gsl:+newton-fdfsolver+))
  (multiple-value-bind (initial-num initial-units)
      (pqval initial)
    (multiple-value-bind (x f)
	(call-next-method
	 (lambda (x) (pqval (funcall function (make-pq x initial-units))))
	 initial-num
	 (lambda (x) (pqval (funcall derivative (make-pq x initial-units))))
	 absolute-error
	 relative-error
	 method)
      (values (make-pq x initial-units) f))))

