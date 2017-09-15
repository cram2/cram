;; Fast fourier transform tests
;; Liam Healy 2010-08-14 11:58:26EDT fast-fourier-transform.lisp
;; Time-stamp: <2011-05-26 12:37:30EDT fast-fourier-transform.lisp>
;;
;; Copyright 2010, 2011 Liam M. Healy
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

(in-package :gsl)

(defvar *allowed-ticks* 1000000)

#+nil
(defun off-stride (array stride)
  "Make a grid that is all of the original array except that the
   stride points are excluded."
  (grid:make-foreign-array
   (grid:element-type array)
   :initial-contents
   (loop for i from 0 below (grid:total-size array)
      unless (zerop (mod i stride))
      collect (grid:aref* array i))))

(defun fft-complex-off-stride-check (vector stride offset)
  (loop with size = (size vector)
	for i from 0 below size
	unless (zerop (mod i stride))
	unless (lisp-unit:numerical-equal
		 (complex (+ offset (* 2 i))
			  (+ offset (* 2 i) 1))
		 (grid:aref vector i))
	return nil
	finally
	(return t)))

(defmacro fft-complex-result-check (form element-component-type stride)
  "T if all FFT tests pass."
  `(multiple-value-bind
	 (dft fft original fft-roundtrip fft-reverse)
       (progn (reset-urand) ,form)
     (let ((lisp-unit:*epsilon*
	    (* *allowed-ticks*
	       ,(if (eq element-component-type 'single-float)
		    '+sgl-epsilon+
		    '+dbl-epsilon+)))
	   (lisp-unit:*measure* :infinity))
       (when (> ,stride 1)
	 (lisp-unit:assert-true (fft-complex-off-stride-check fft ,stride 2000))
	 (lisp-unit:assert-true (fft-complex-off-stride-check fft-roundtrip ,stride 0))
	 (lisp-unit:assert-true (fft-complex-off-stride-check fft-reverse ,stride 2000))
	 (setf dft (grid:stride dft ,stride)
	       fft (grid:stride fft ,stride)
	       original (grid:stride original ,stride)
	       fft-roundtrip (grid:stride fft-roundtrip ,stride)
	       fft-reverse (grid:stride fft-reverse ,stride)))
       (lisp-unit:assert-norm-equal dft fft)
       (lisp-unit:assert-norm-equal original fft-roundtrip)
       (lisp-unit:assert-norm-equal original fft-reverse))))

(defmacro fft-real-result-check (form element-type stride)
  "T if all FFT tests pass."
  `(multiple-value-bind
	 (dft fft original fft-roundtrip)
       (progn (reset-urand) ,form)
     (when (> ,stride 1)
       (setf dft (grid:stride dft ,stride)
             fft (grid:stride fft ,stride)
             original (grid:stride original ,stride)
             fft-roundtrip (grid:stride fft-roundtrip ,stride)))
     (let ((lisp-unit:*epsilon*
	    (* *allowed-ticks*
	       ,(if (eq element-type 'single-float)
		    '+sgl-epsilon+
		    '+dbl-epsilon+)))
	   (lisp-unit:*measure* :infinity))
       (lisp-unit:assert-norm-equal dft fft)
       (lisp-unit:assert-norm-equal original fft-roundtrip))))

(eval-when (:compile-toplevel :load-toplevel :execute)
  (defun fft-test-forms (size stride)
    `((fft-real-result-check
       (test-fft-noise 'double-float ,size :stride ,stride :non-radix-2 t) double-float ,stride)
      (fft-real-result-check
       (test-fft-noise 'single-float ,size :stride ,stride :non-radix-2 t) single-float ,stride)
      (fft-complex-result-check
       (test-fft-noise
	'(complex double-float) ,size :stride ,stride :non-radix-2 t) double-float ,stride)
      (fft-complex-result-check
       (test-fft-noise
	'(complex single-float) ,size :stride ,stride :non-radix-2 t) single-float ,stride)
      (when (power-of-2-p (floor ,size ,stride))
	(fft-real-result-check
	  (test-fft-noise 'double-float ,size :stride ,stride :non-radix-2 nil) double-float ,stride)
	(fft-complex-result-check
	  (test-fft-noise
	    '(complex double-float) ,size :stride ,stride :non-radix-2 nil) double-float ,stride)))))

(defmacro all-fft-test-forms (size-max stride-max &optional additional-single-stride)
  `(lisp-unit:define-test fast-fourier-transform
     ,@(loop for size from 1 to size-max
	  append
	  (loop for stride from 1 to stride-max
	     append (fft-test-forms size stride)))
     ,@(when additional-single-stride
	     (loop for size in additional-single-stride
		append
		(fft-test-forms size 1)))))


;(all-fft-test-forms 20 3)
;;; Tests commented out because they come out not so good.
