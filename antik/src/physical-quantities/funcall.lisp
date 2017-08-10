;; Generic dyadic calls
;; Liam Healy 2011-02-15 21:30:46EST funcall.lisp
;; Time-stamp: <2013-11-23 12:15:36EST funcall.lisp>

;;; This must be defined here because grid type must be loaded.

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

(defun funcall-dyadic (op a b &optional return-other return-other-monadic)
  "Call the math function on two arguments a and b, possibly
   converting one argument from a number to a generic number like the
   other.  If return-other is a function designator, funcall
   return-other-monadic on the other argument if one passes that
   test."
  ;; This should not be used if both a and b are numbers.  Check that
  ;; both arguments are not non-pq grids, otherwise it will recurse
  ;; indefinitely.
  (assert (not (and (grid:gridp a) (not (physical-quantity-p a))
		    (grid:gridp b) (not (physical-quantity-p b))))
	  (op a b)
	  "There is no known way to apply ~a to arguments of type ~a and ~a."
	  op (type-of a) (type-of b))
  (cond
    ((and return-other (funcall return-other a))
     (funcall (or return-other-monadic 'identity) b))
    ((and return-other (funcall return-other b))
     (funcall (or return-other-monadic 'identity) a))
    ((and (physical-quantity-p a) (not (grid:gridp a)) (grid:gridp b))
     (grid:map-grid
      :source b
      :element-function (lambda (x) (funcall op a x))
      :destination-specification
      (grid::pdq-grid-specification (grid:specification b))))
    ((and (physical-quantity-p b) (not (grid:gridp b)) (grid:gridp a))
     (grid:map-grid
      :source a
      :element-function (lambda (x) (funcall op x b))
      :destination-specification
      (grid::pdq-grid-specification (grid:specification a))))
    (t
     (handler-case
	 (funcall op
		  ;; if args are pdqs, just throw away units
		  (etypecase a
		    (number (coerce a b))
		    (physical-quantity (pqval a))
		    (grid:grid a))
		  (etypecase b
		    (number (coerce b a))
		    (physical-quantity (pqval b))
		    (grid:grid b)))
       (coerce-undefined ()
	 ;; This is redundant, but I'm leaving it here so that it
	 ;; could be replaced with something more meaningful if
	 ;; necessary.  Can't use call-next-method here.
	 (error "Don't know how to coerce ~a and ~a." a b))))))

(defmethod +i (a b)
  (funcall-dyadic '+i a b 'zerop))

(defmethod -i  (a b)
  (funcall-dyadic
   '-i a b (lambda (x) (or (null x) (zerop x))) (lambda (x) (* -1 x))))

(defmethod *i (a b) (funcall-dyadic '*i a b))

(defmethod /i (a b) (funcall-dyadic '/i a b 'null (lambda (x) (/ 1 x))))
