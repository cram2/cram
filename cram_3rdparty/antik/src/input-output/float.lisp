;; Floating point numbers.
;; Liam Healy Wed May  8 2002 - 10:47
;; Time-stamp: <2015-04-25 12:11:26EDT float.lisp>

;; Copyright 2011, 2012 Liam M. Healy
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

(export '(*format-float-significant-figures*))

(defparameter *print-sign* nil
  "Always print the sign, even if +.")

;;; There apparently is no introspection on format's default number of
;;; digits; the best we can do is a little experimentation.
(defconstant +cl-float-format-default-fractional-digits+
  (- (length (format nil "~f" pi)) 2))

(defun format-float-fixed
    (float
     &optional (intpart-digits 2) (fracpart-digits 3) (intpart-fill #\space)
     (print-sign *print-sign*))
  "Format a floating-point number with the specified number of digits in the
   fractional (and optionally) integer parts, optionally filling the integer part."
  (let ((fpd (or fracpart-digits +cl-float-format-default-fractional-digits+)))
    (if (plusp fpd)
	;; print with digits after the decimal
	(format nil
		(if print-sign "~v,v,,,v@f" "~v,v,,,vf")
		(+ 1
		   (if (or print-sign (minusp float)) 1 0) ; an extra space for sign
		   (or intpart-digits 1)
		   fpd)
		fpd
		intpart-fill
		float)
	;; print like an integer (no decimal point)
	(format nil
		(if print-sign "~v,v@d" "~v,vd")
		(or intpart-digits 0)
		intpart-fill
		(round float)))))

;;; (format-float-fixed 3)
;;; " 3.000"
;;; (format-float-fixed 3 2 0 #\0)
;;; "03"
;;; (format-float-fixed 23)
;;; "23.000"
;;; (format-float-fixed 23.0)
;;; "23.000"
;;; (format-float-fixed 3.4300)
;;; "3.43"
;;; (format-float-fixed 4.56 2 3 #\0)
;;; "04.560"
;;; (format-float-fixed 4.56 3 3)
;;; "  4.560"

#+test
(loop for exp from -8 upto 8
    do (print (format-float-fixed (* pi (expt 10 exp)))))
;;; "0" 
;;; "0" 
;;; "0.000003" 
;;; "0.000031" 
;;; "0.000314" 
;;; "0.003142" 
;;; "0.031416" 
;;; "0.314159" 
;;; "3.141593" 
;;; "31.415927" 
;;; "314.159265" 
;;; "3141.592654" 
;;; "31415.926536" 
;;; "314159.265359" 
;;; "3141592.65359" 
;;; "31415926.535898" 
;;; "314159265.358979" 

(defparameter *format-float-significant-figures* 5
  "The default number of significant figures in
   floating point number output.")

(defun format-float-switch
    (float &optional
	     (significant-figures *format-float-significant-figures*)
	     (print-sign *print-sign*)
	     (specific-exponent-marker t))
  "Format a floating point number for printing in a readable way.  If specific-exponent-marker is T use the 'd', 'f' etc. to mark the exponent, instead of 'e'."
  (string-right-trim
   '(#\.)
   (string-trim
    '(#\ )
    (format nil
	    (if print-sign "~v,v,,,,,v@g" "~v,v,,,,,vg")
	    (cl:+ significant-figures
		  (if (or print-sign (cl:minusp float)) 7 6))
	    (cl:+ significant-figures
		  ;; compensate for misdesign of ~g and subtract
		  ;; an extra digit of precision in exponential form,
		  ;; so the same number of significant digits are printed in fixed
		  ;; and exponential forms.
		  (if (or (zerop float)
			  (cl:<= 0
				 (1+ (cl:floor (cl:log (cl:abs float) 10)))
				 significant-figures))
		      0 -1))
	    (if specific-exponent-marker
		(etypecase float
		  (short-float #\s)
		  (single-float #\f)
		  (double-float #\d)
		  (long-float #\l))
		#\e)
	    float))))

;;; (loop for exp from -8 upto 8 do (print (format-float-switch (* pi (expt 10 exp)) 6)))
;;; "3.14159e-8" 
;;; "3.14159e-7" 
;;; "3.14159e-6" 
;;; "3.14159e-5" 
;;; "3.14159e-4" 
;;; "3.14159e-3" 
;;; "3.14159e-2" 
;;; "0.314159" 
;;; "3.14159" 
;;; "31.4159" 
;;; "314.159" 
;;; "3141.59" 
;;; "31415.9" 
;;; "314159." 
;;; "3.14159e+6" 
;;; "3.14159e+7" 
;;; "3.14159e+8" 

;;; (loop for exp from -8 upto 8 do (print (format-float-fixed (* pi (expt 10 exp)) nil 5)))
;;; "0.00000" 
;;; "0.00000" 
;;; "0.00000" 
;;; "0.00003" 
;;; "0.00031" 
;;; "0.00314" 
;;; "0.03142" 
;;; "0.31416" 
;;; "3.14159" 
;;; "31.41593" 
;;; "314.15927" 
;;; "3141.59265" 
;;; "31415.92654" 
;;; "314159.26536" 
;;; "3141592.65359" 
;;; "31415926.53590" 
;;; "314159265.35898" 

;;; (loop for exp from -8 upto 8 do (print (format-float-fixed (* pi (expt 10 exp)) 7 5)))
;;; "      0.00000" 
;;; "      0.00000" 
;;; "      0.00000" 
;;; "      0.00003" 
;;; "      0.00031" 
;;; "      0.00314" 
;;; "      0.03142" 
;;; "      0.31416" 
;;; "      3.14159" 
;;; "     31.41593" 
;;; "    314.15927" 
;;; "   3141.59265" 
;;; "  31415.92654" 
;;; " 314159.26536" 
;;; "3141592.65359" 
;;; "31415926.53590" 
;;; "314159265.35898" 

(defun format-float
    (float &key
     (significant-figures *format-float-significant-figures*)
     intpart-digits fracpart-digits (intpart-fill #\space)
     (print-sign *print-sign*))
  "Format a floating point number with control over the digits that are printed.
   If significant-figures is specified, the formating is as if by ~g, and the given
   number of significant figures are formated if using plain float formatting,
   one extra significant digit is formated if in scientific notation.  If
   intpart-digits/fracpart-digits are specified, formatting will always be
   in plain format, with the number of digits to the left and right controlled
   by these parameters."
  (if significant-figures
      (format-float-switch float significant-figures print-sign)
      (format-float-fixed
       (if (typep float 'rational)
	   (cl:coerce float *read-default-float-format*)
	   float)
       intpart-digits fracpart-digits
       intpart-fill
       print-sign)))

;; Used to be:
;; significant-figures=nil and fracpart-digits=nil means print
;; CL's default (full precision) format
;; (princ-to-string float)

;;;;****************************************************************************
;;;; nf
;;;;****************************************************************************

(define-parameter nf :fracpart-digits
  :value 3 :type (or null fixnum)
  :documentation
  "The number of digits to the right of the decimal floating point numbers
   when :significant-figures is nil.")

(define-parameter nf :full-precision
  :value nil :type t
  :documentation
  "If non-nil, ignore :fracpart-digits, :intpart-digits, :significant-figures,
   and format floating point numbers to full precision.")

(defmethod nf
  ((object float) &optional (stream *standard-output*))
  "Returns two values: the string and prints-as-zero."
  (let ((plain 
	 (format-float
	  object
	  :significant-figures
	  (if (nf-option :full-precision) nil (nf-option :significant-figures))
	  :fracpart-digits 
	  (if (nf-option :full-precision) nil (nf-option :fracpart-digits))
	  :intpart-digits (nf-option :intpart-digits)
	  :print-sign (nf-option :print-sign))))
    (values
      (case (nf-option :style)
	(:tex
	   (let ((emarker		; position of "e" if any
		  (position-if #'alpha-char-p plain)))
	     (if emarker
		 ;; scientific notation
		 (format stream "~a \\times 10^{~d}"
			 (subseq plain 0 emarker)
			 (read-from-string (subseq plain (1+ emarker))))
		 ;; fixed format
		 (princn plain stream))))
	(:readable
	   (if (find-if #'alpha-char-p plain) ; already has an exponent marker
	       (princn plain stream)
	       (format stream "~a~a0"
		       plain
		       (etypecase object
			 (double-float #\d)
			 (single-float #\f)))))
	(t ; style=NIL, print plain
	   (princn plain stream)))
      (zerop (read-from-string plain)))))

