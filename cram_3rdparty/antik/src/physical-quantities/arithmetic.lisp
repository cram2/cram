;; Arithmetic with physical dimension quantities.
;; Liam Healy Fri Apr 22 2005 - 09:58
;; Time-stamp: <2015-12-17 15:49:44EST arithmetic.lisp>

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

(export '(flatten-angle))

(defgeneric flatten-angle (object)
  (:documentation
   "Remove the angular dimension; the number is not changed
   and therefore the angles are radians.  This function is
  necessary as proper dimensional arithmetic ignores angles as a
  dimension.")
  (:method ((object physical-quantity))
    (make-pq-object
     (pq-magnitude object)
     (let ((dimel-flat (copy-list (pq-dimension object))))
       (setf (nth (position 'angle *basis-dimensions*)
		  dimel-flat)
	     0)
       dimel-flat)
     (scalar-dimension object)))
  (:method ((object sequence))
    (map (type-of object) #'flatten-angle object))
  (:method (object) object))

(defmacro grid-call-next-method (&rest args)
  `(when (or ,@(mapcar (lambda (x) (list 'grid:gridp x)) args))
     (call-next-method)))

(defmethod *i ((a physical-quantity) (b physical-quantity))
  (undimensioned-should-not-be-here)
  ;;(grid-call-next-method a b)
  (make-pq-object
   (* (pq-magnitude a) (pq-magnitude b))
   (op-dimension (pq-dimension a) (pq-dimension b))
   (and (scalar-dimension a) (scalar-dimension b))))

(defmethod *i ((a physical-quantity) (b number))
  (undimensioned-should-not-be-here)
  (make-pq-object
   (* (pq-magnitude a) b)
   (pq-dimension a)
   (scalar-dimension a)))

(defmethod *i ((a number) (b physical-quantity))
  (undimensioned-should-not-be-here)
  (make-pq-object
   (* a (pq-magnitude b))
   (pq-dimension b)
   (scalar-dimension b)))

(defmethod /i ((a physical-quantity) (b physical-quantity))
  (undimensioned-should-not-be-here)
  ;;(grid-call-next-method a b)
  (make-pq-object
   (/ (pq-magnitude a) (pq-magnitude b))
   (op-dimension (pq-dimension a) (pq-dimension b) #'-)
   (and (scalar-dimension a) (scalar-dimension b))))

(defmethod /i ((a physical-quantity) (b number))
  (undimensioned-should-not-be-here)
  (make-pq-object
   (/ (pq-magnitude a) b)
   (pq-dimension a)
   (scalar-dimension a)))

(defmethod /i ((a number) (b physical-quantity))
  (undimensioned-should-not-be-here)
  (make-pq-object
   (/ a (pq-magnitude b))
   (antik:- (pq-dimension b))
   (scalar-dimension b)))

(defmethod +i ((a physical-quantity) (b physical-quantity))
  (undimensioned-should-not-be-here)
  ;;(grid-call-next-method a b)
  (multiple-value-bind (dimel scalardim)
      (equal-dimension a b)
    (make-pq-object
     (+i (pq-magnitude a) (pq-magnitude b))
     dimel
     scalardim)))

(defmethod +i ((a number) (b physical-quantity))
  (undimensioned-should-not-be-here)
  (if (cl:zerop a)
      b
      (let ((flatb (flatten-angle b)))
	(if (numberp flatb)
	    (cl:+ a flatb)
	    (call-next-method)))))

(defmethod +i ((a physical-quantity) (b number))
  (undimensioned-should-not-be-here)
  (if (cl:zerop b)
      a
      (let ((flata (flatten-angle a)))
	(if (numberp flata)
	    (cl:+ flata b)
	    (call-next-method)))))

(defmethod -i ((a physical-quantity) (b physical-quantity))
  (undimensioned-should-not-be-here)
  ;;(grid-call-next-method a b)
  (multiple-value-bind (dimel scalardim)
      (equal-dimension a b)
    (make-pq-object
     (- (pq-magnitude a) (pq-magnitude b))
     dimel
     scalardim)))

(defmethod -i ((a number) (b physical-quantity))
  (undimensioned-should-not-be-here a b)
  (if (cl:zerop a)			; just changing the sign
      (call-next-method)
      (let ((flatb (flatten-angle b)))
	(if (numberp flatb)
	    ;; Subtracting an angle from a number means we need to
	    ;; turn everything into radians.
	    (cl:- a flatb)
	    (call-next-method)))))

(defmethod -i ((a physical-quantity) (b number))
  (undimensioned-should-not-be-here)
  (if (cl:zerop b)
      a
      (let ((flata (flatten-angle a)))
	(if (numberp flata)
	    (cl:- flata b)
	    ;; Subtracting a number from an angle means we need to
	    ;; turn everything into radians.
	    (call-next-method)))))

;;; Why does this exist?
#+remove
(defmethod g-i :around ((a number) (b physical-quantity))
  (let ((*zero-is-dimensionless* nil))
    (call-next-method)))

(defmethod sin ((x physical-quantity))
  (undimensioned-should-not-be-here)
  (cl:sin (flatten-angle x)))

(defmethod cos ((x physical-quantity))
  (undimensioned-should-not-be-here)
  (cl:cos (flatten-angle x)))

(defmethod tan ((x physical-quantity))
  (undimensioned-should-not-be-here)
  (cl:tan (flatten-angle x)))

(defmethod sinh ((x physical-quantity))
  (undimensioned-should-not-be-here)
  (cl:sinh (flatten-angle x)))

(defmethod cosh ((x physical-quantity))
  (undimensioned-should-not-be-here)
  (cl:cosh (flatten-angle x)))

(defmethod tanh ((x physical-quantity))
  (undimensioned-should-not-be-here)
  (cl:tanh (flatten-angle x)))

(defmethod asin ((x physical-quantity))
  ;; This should always signal an error
  (check-dimension x 'dimensionless))

(defmethod acos ((x physical-quantity))
  ;; This should always signal an error
  (check-dimension x 'dimensionless))

(defmethod atan ((num physical-quantity)
		 &optional (den 1) (default 0)
		 (zero-enough (* 64 double-float-epsilon)))
  (equal-dimension num den)
  (make-pq
   (atan (pqval num) (pqval den) default zero-enough)
   'radian))

(defmethod atan :around ((num number)
			 &optional (den 1) (default 0)
			   (zero-enough (* 64 double-float-epsilon)))
  (if (physical-quantity-p den)
      (atan
       (let ((*zero-is-dimensionless* nil)) (accept-coerce (coerce num den)))
       den default zero-enough)
      (call-next-method)))

(defmethod coerce ((num number) (pq physical-quantity))
  (cerror "Accept"
	  "Coercing number ~a to physical dimension quantity ~a"
	  num (nth-value 1 (pqval pq)))
  (make-pq num (nth-value 1 (pqval pq))))

(defmethod expt ((x physical-quantity) expon)
  (undimensioned-should-not-be-here)
  (make-pq-object
   (cl:expt (pq-magnitude x) expon)
   (antik:* expon (pq-dimension x))
   (scalar-dimension x)))

(defmethod abs ((x physical-quantity))
  (undimensioned-should-not-be-here)
  (make-pq-object
   (cl:abs (pq-magnitude x))
   (pq-dimension x)
   (scalar-dimension x)))

(defmethod zerop ((x physical-quantity))
  (zerop (pq-magnitude x)))

(defmethod minusp ((x physical-quantity))
  (undimensioned-should-not-be-here)
  (minusp (pq-magnitude x)))

(defmethod plusp ((x physical-quantity))
  (undimensioned-should-not-be-here)
  (plusp (pq-magnitude x)))

(defmethod floor ((x physical-quantity) &optional (y 1))
  (undimensioned-should-not-be-here)
  (antik:floor (pqval x) (pqval y)))

(defmethod numbcomp ((x physical-quantity))
  (undimensioned-should-not-be-here)
  (pq-magnitude x))

#|
(defmethod -nz
    ((x physical-quantity) (y physical-quantity)
     &optional (relative-difference 2))
  (undimensioned-should-not-be-here)
  (equal-dimension x y)
  (-nz (pqval x) (pqval y) relative-difference))
|#

