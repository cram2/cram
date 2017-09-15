;; Basic elementwise mathematics on grids.
;; Liam Healy 2011-01-03 21:04:20EST mathematics.lisp
;; Time-stamp: <2011-06-07 09:28:56EDT mathematics.lisp>

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

;;; These are elementary matematical operations.  At present, the
;;; scalars must be numbers and the components of arrays must also be
;;; numbers.  This might be broadened later.

(in-package :antik)

(export '(def-monadic def-dyadic-scalar))

(defmacro def-monadic (operation)
  "Define a method of the operation that operates elementwise on a grid."
  `(defmethod ,operation ((x t))
     (if (grid:gridp x)
	 (funcall (grid:elementwise ',operation) x)
	 (call-next-method))))

(def-monadic sin)
(def-monadic cos)
(def-monadic tan)
(def-monadic sinh)
(def-monadic cosh)
(def-monadic tanh)
(def-monadic asin)
(def-monadic acos)
(def-monadic exp)
(def-monadic abs)

;;; Note that GSLL duplicates some of this functionality for
;;; foreign-array in antik/linear-algebra.lisp.

(defmacro def-dyadic-scalar (method &optional (operation method) (left t) (right t))
  "Define a method of the operation that operates elementwise on a grid."
  `(progn
     ,@(when right
	 `((defmethod ,method ((a t) (b number))
	     (if (grid:gridp a)
		 (funcall
		  (grid:elementwise (alexandria:rcurry ',operation b))
		  a)
		 (call-next-method)))))
     ,@(when left
	 `((defmethod ,method ((a number) (b t))
	     (if (grid:gridp b)
		 (funcall
		  (grid:elementwise (alexandria:curry ',operation a))
		  b)
		 (call-next-method)))))))

(def-dyadic-scalar *i)
(def-dyadic-scalar /i)
(def-dyadic-scalar +i)
(def-dyadic-scalar -i)
(def-dyadic-scalar expt expt nil)

(defmethod log ((a t) &optional base)
  (if (grid:gridp a)
      (if base 
	  (grid:map-grid
	   :source a
	   :element-function
	   (alexandria:rcurry 'cl:log base))
	  (grid:map-grid
	   :source a
	   :element-function 'cl:log))
      (call-next-method)))

(defmethod atan
  ((num t) &optional (den 1) (default 0) (zero-enough (* 64 double-float-epsilon)))
  (if (grid:gridp num)
      (grid:map-grid
       :source num
       :element-function
       (alexandria:rcurry 'atan den default zero-enough))
      (call-next-method)))
