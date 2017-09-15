;; Generic functions arithmetic functions on generalized numbers
;; Liam Healy Tue Feb  9 1999 - 21:38
;; Time-stamp: <2015-11-15 15:53:03EST generic.lisp>

;; Copyright 2011, 2013, 2014, 2015 Liam M. Healy
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

;;;;****************************************************************************
;;;; Convert numeric types
;;;;****************************************************************************

(defgeneric coerce (number like)
  (:documentation "Make number into an object similar to like, or
                   if like is a number type, into that type.")
  (:method (x like) (error 'coerce-undefined :object x :to like))
  (:method ((number number) (like number))
	   (cl:coerce number (type-of like)))
  (:method ((number symbol) (like number))
	   (error "Can't coerce symbol."))
  (:method (number (like symbol))
	   (if (eq like 't)
	       number
	     (if (subtypep like 'number)
		 (cl:coerce number like)
	       (error "Don't know how to coerce to ~a" like))))
  (:method ((number sequence) like)
	   (map (type-of number) (lambda (x) (coerce x like)) number))
  (:method (object (like null))
    (error 'coerce-nil :object object)))

(defgeneric gconstant (name like)
  (:method ((name (eql 'pi)) like)
	   (coerce pi like)))

;;; (with-converted-numbers ('double-float x y z) (fuzzle x y z))
(defmacro with-converted-numbers ((like &rest symbols) &body body)
  (let ((lets 
	 (mapcar (lambda (x) (list x `(coerce ,x ,like))) symbols)))
    ;; can't us pi here, error is:
    ;; Error: SYMBOL-MACROLET symbol PI is globally declared SPECIAL
    `(symbol-macrolet ((gpi (gconstant 'pi ,like)))
       (let ,lets
	 ,@body))))

;;;;****************************************************************************
;;;; Dyadic functions
;;;;****************************************************************************

;;; There is a general problem with using coerce in these functions;
;;; if the second argument is T, the first argument will be returned
;;; and the function will recurse infinitely.  
(defgeneric +i (a b)
  (:documentation "Addition of generalized or regular numbers.")
  (:method ((a number) (b number)) (cl:+ a b)))

(defun + (&rest args)
  "Addition of generalized or regular numbers with an arbitrary number of arguments."
  (when (member nil args) (warn 'null-argument))
  (if (single args) (first args) (reduce #'+i args)))

(defgeneric -i (a b)
  (:documentation
   "Subtraction/negation of generalized or regular numbers, internal.
    Users call -.")
  (:method ((a number) (b number)) (cl:- a b)))

(defun - (&rest args)
  "Subtraction of generalized or regular numbers."
  (when (member nil args) (warn 'null-argument))
  (if (single args) (*i -1 (first args)) (reduce #'-i args)))

(defgeneric *i (a b)
  (:documentation "Multiplication of generalized or regular numbers.")
  (:method ((a number) (b number)) (cl:* a b)))

(defun * (&rest args)
  "Multiplication of generalized or regular numbers with an arbitrary number of arguments."
  (when (member nil args) (warn 'null-argument))
  (if (single args) (first args) (reduce #'*i args)))

(defgeneric /i (a b)
  (:documentation "Division of generalized or regular numbers, internal.
    Users call /.")
  (:method ((a number) (b number)) (cl:/ a b)))

(defun / (&rest args)
  "Division of generalized or regular numbers for an arbitrary number of arguments."
  (when (member nil args) (warn 'null-argument))
  (if (single args)
      (/ 1 (first args))
      (reduce #'/i args)))

;;; Put these into with-generalized numbers?
(defmacro incf (ref &optional (delta 1))
  `(setf ,ref (+i ,ref ,delta)))

(defmacro decf (ref &optional (delta 1))
  `(setf ,ref (- ,ref ,delta)))

;;;;****************************************************************************
;;;; Monadic functions
;;;;****************************************************************************

(defun sqrt (num)
  "The square root :math:`\\sqrt{x}` of the generalized or regular number :math:`x`."
  (expt num 1/2))

(defgeneric sin (num)
  (:documentation "The sine of the generalized or regular number.")
  (:method ((num rational))
	   (cl:sin (coerce num *read-default-float-format*)))
  (:method ((num number))
	   (cl:sin num))
  (:method ((x sequence))
	   (map (type-of x) #'sin x)))

(defgeneric cos (num)
  (:documentation "The cosine of the generalized or regular number.")
  (:method ((num rational))
	   (cl:cos (coerce num *read-default-float-format*)))
  (:method ((num number))    
	   (cl:cos num))
  (:method ((x sequence))
	   (map (type-of x) #'cos x)))

(defgeneric tan (num)
  (:documentation "The tangent of the generalized or regular number.")
  (:method ((num rational))
	   (cl:tan (coerce num *read-default-float-format*)))
  (:method ((num number))
	   (cl:tan num))
  (:method ((x sequence))
	   (map (type-of x) #'tan x)))

(defparameter *radian* 1)
(defun make-radian (x)
  "From the number x, return the representation of an angle in radians."
  (* *radian* x))

(defgeneric asin (arg)
  (:documentation "The arcsine of the generalized or regular number.")
  (:method ((arg number))
    (when (and (not (complexp arg)) (> (abs arg) 1))
      (restart-case 
	  (error 'making-complex-number :operation 'asin :number arg)
	(accept () :report "Accept complex answer.")
	(truncate ()
	  :report "Truncate argument to ±1."
	  (return-from asin
	    (if (plusp arg) (cl:asin 1.0d0) (cl:asin -1.0d0))))
	(hyperbolic-function ()
	  :report "Use the inverse hyperbolic sine."
	  (return-from asin (cl:asinh arg)))))
    (make-radian (cl:asin arg))))

(defgeneric acos (arg)
  (:documentation "The arccosine of the generalized or regular number.")
  (:method ((arg number))
    (when (and (not (complexp arg)) (> (abs arg) 1))
      (restart-case 
	  (error 'making-complex-number :operation 'acos :number arg)
	(accept () :report "Accept complex answer.")
	(truncate ()
	  :report "Truncate argument to ±1."
	  (return-from acos
	    (if (plusp arg) (cl:acos 1.0d0) (cl:acos -1.0d0))))
	(hyperbolic-function ()
	  :report "Use the inverse hyperbolic cosine."
	  (return-from acos (cl:acosh arg)))))
    (make-radian (cl:acos arg))))

(defgeneric atan (num &optional den default zero-enough)
  (:documentation "The arctangent of the generalized or regular number.  If  absolute value of the arguments are below zero-enough, default is returned.")
  (:method ((num number)
	    &optional (den 1) (default 0) (zero-enough (* 64 double-float-epsilon)))
	   ;; zero-enough should be keyed to epsilon of num type
    (make-radian
     (if (and (< (cl:abs num) zero-enough) (< (cl:abs den) zero-enough))
	 default
	 (cl:atan (if (typep num 'rational)
		      (coerce num *read-default-float-format*)
		      num)
		  (if (zerop num)
		      (signum den)
		      den))))))

(export 'return-zero)
(defgeneric expt (num exponent)
  (:documentation "Raise the number to the exponent.")
  (:method ((num number) (exponent number))
    ;; Most calculations we do are real,
    ;; so trap the production of complex numbers.
    ;; Presumably if the number is already complex,
    ;; we don't mind complex numbers.
    (when (and (not (complexp num)) (minusp num) (not (integerp exponent)))
      (restart-case 
	  (error 'making-complex-number :operation 'expt :number num)
	(accept () :report "Accept complex answer.")
	(absolute-value ()
	  :report "Take the absolute value of the argument and return a real."
	  (setq num (abs num)))
	(return-zero ()
	  :report "Return zero."
	  (return-from expt 0.0))))
    (cl:expt num exponent)))

(defgeneric exp (num)
  (:documentation "The natural exponent e^num of the generalized or regular number.")
  (:method ((num rational))
	   (cl:exp (coerce num *read-default-float-format*)))
  (:method ((num number))
	   (cl:exp num))
  (:method ((x sequence))
	   (map (type-of x) #'exp x)))

(defgeneric log (num &optional base)
  (:documentation "The natural logarithm of the generalized or regular number.")
  (:method ((num number) &optional base)
    (if base
	(cl:log num base)
	(cl:log num)))
  (:method ((x sequence) &optional base)
    (if base
	(map (type-of x) (alexandria:rcurry 'log base) x)
	(map (type-of x) #'log x))))

(defgeneric abs (num)
  (:documentation "The absolute value.")
  (:method ((num number))
	   (cl:abs num))
  (:method ((seq sequence))
	   (map (type-of seq) #'abs seq)))

(defgeneric sinh (num)
  (:documentation "The hyperbolic sine of the generalized or regular number.")
  (:method ((num rational))
	   (cl:sinh (coerce num *read-default-float-format*)))
  (:method ((num number))
	   (cl:sinh num))
  (:method ((x sequence))
	   (map (type-of x) #'sinh x)))

(defgeneric cosh (num)
  (:documentation "The hyperbolic cosine of the generalized or regular number.")
  (:method ((num rational))
	   (cl:cosh (coerce num *read-default-float-format*)))
  (:method ((num number))    
	   (cl:cosh num))
  (:method ((x sequence))
	   (map (type-of x) #'cosh x)))

(defgeneric tanh (num)
  (:documentation "The hyperbolic tangent of the generalized or regular number.")
  (:method ((num rational))
	   (cl:tanh (coerce num *read-default-float-format*)))
  (:method ((num number))
	   (cl:tanh num))
  (:method ((x sequence))
	   (map (type-of x) #'tanh x)))

;;;;****************************************************************************
;;;; Comparisons
;;;;****************************************************************************

(defgeneric numbcomp (x)
  (:documentation "Function for number comparison.  If this function is
   defined for a generalized number and returns an object for which all
   comparison functions such as <, >, etc. are defined,
   then all comparison functions will automatically be defined
   for that object.")
  (:method ((x number)) x))

(defun multiarg-comparison (op args)
  "Extend a comparison operator from two to any number of arguments."
  (every op args (rest args)))

(defun >= (&rest args) (multiarg-comparison '>=i args))
(defgeneric >=i (a b)
  (:documentation "Greater than or equal.")
  (:method ((a number) (b number))
	   (cl:>= a b))
  (:method (a b) (cl:>= (numbcomp a) (numbcomp b))))

(defun > (&rest args) (multiarg-comparison '>i args))
(defgeneric >i (a b)
  (:documentation "Greater than")
  (:method ((a number) (b number)) (cl:> a b))
  (:method (a b) (cl:> (numbcomp a) (numbcomp b))))

(defun <= (&rest args) (multiarg-comparison '<=i args))
(defgeneric <=i (a b)
  (:documentation "Less than or equal.")
  (:method ((a number) (b number))
	   (cl:<= a b))
  (:method (a b) (cl:<= (numbcomp a) (numbcomp b))))

(defun < (&rest args) (multiarg-comparison '<i args))
(defgeneric <i (a b)
  (:documentation "Less than.")
  (:method ((a number) (b number)) (cl:< a b))
  (:method (a b) (cl:< (numbcomp a) (numbcomp b))))

(defun = (&rest args) (multiarg-comparison '=i args))
(defgeneric =i (a b)
  (:documentation "Numeric equal")
  (:method ((a number) (b number))
	   (if (cl:= a b) a))
  #+(or) 
  (:method (a b) (funcall-dyadic '= (numbcomp a) (numbcomp b))))

(defgeneric plusp (a)
  (:documentation "Positive.")
  (:method ((a number))
	   (cl:plusp a))
  (:method (a) (cl:plusp (numbcomp a))))

(defgeneric minusp (a)
  (:documentation "Negative")
  (:method ((a number))
	   (cl:minusp a))
  (:method (a) (cl:minusp (numbcomp a))))

(defgeneric zerop (a)
  (:documentation "Zero.")
  (:method ((a number))
	   (cl:zerop a))
  (:method ((x sequence))
	   (and x (every #'zerop x)))
  (:method (a) (cl:zerop (numbcomp a))))

(defgeneric floor (number &optional divisor)
  (:documentation "Greatest multiple of divisor less than number.")
  (:method ((number number) &optional (divisor 1))
	   (cl:floor number divisor)))

(defgeneric round (number &optional divisor)
  (:documentation "Nearest multiple of divisor to number.")
  (:method ((number number) &optional (divisor 1))
	   (cl:round number divisor)))

(defgeneric signum (a)
  (:documentation "Signum(a).")
  (:method ((a number)) (cl:signum a))
  (:method (a) (cl:signum (numbcomp a))))

(defun min (&rest args)
  (reduce (lambda (x y) (if (< x y) x y)) args))

(defun max (&rest args)
  (reduce (lambda (x y) (if (> x y) x y)) args))
