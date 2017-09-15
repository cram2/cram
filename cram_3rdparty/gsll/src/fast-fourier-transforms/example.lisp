;; Example FFT: transform a pulse (using the "clean" fft interface)
;; Sumant Oemrawsingh, Sat Oct 31 2009 - 00:24
;; Time-stamp: <2011-05-26 12:37:35EDT example.lisp>
;;
;; Copyright 2009, 2010, 2011 Sumant Oemrawsingh, Liam M. Healy
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

;;;;****************************************************************************
;;;; Pulse test
;;;;****************************************************************************

;;; Here is an example program modelled after the example given in Section
;;; 15.3 of the GSL Manual, which computes the FFT of a short pulse. To make
;;; the resulting fourier transform real the pulse is defined for equal
;;; positive and negative times (-10 ... 10), where the negative times wrap
;;; around the end of the array.
;;; 
;;; The output array from the example in Section 15.3 of the GSL Manual can be
;;; reproduced with:
;;; (fft-pulse-test '(complex double-float) 128)
;;;
;;; This example program also yields the same output array as the example
;;; program in Section 15.4 of the GSL Manual:
;;; (fft-pulse-test '(complex double-float) 630)

(defun fft-pulse-test (element-type dimension)
  (assert (and (integerp dimension) (> dimension 20)))
  (let ((pulse (grid:make-foreign-array element-type :dimensions dimension))
        (init-value (coerce 1 element-type)))
    (setf (grid:aref pulse 0) init-value)
    (loop for i from 1 to 10
          do (setf (grid:aref pulse i) init-value
                   (grid:aref pulse (- dimension i)) init-value))
    (forward-fourier-transform pulse)))

(save-test
  fft-pulse
  (fft-pulse-test '(complex single-float) 128)
  (fft-pulse-test '(complex single-float) 630)
  (fft-pulse-test '(complex double-float) 128)
  (fft-pulse-test '(complex double-float) 630)
  (fft-pulse-test 'single-float 128)
  (fft-pulse-test 'single-float 630)
  (fft-pulse-test 'double-float 128)
  (fft-pulse-test 'double-float 630))

;;;;****************************************************************************
;;;; Random vector transformations, from the GSL tests
;;;;****************************************************************************

;; From gsl-1.11/fft/urand.c
(let ((urand-seed 1))
  (defun urand ()
    "Generate a random number.  See fft/urand.c."
    (setf urand-seed (logand #x7fffffff (+ 12345 (* urand-seed 1103515245))))
    (/ urand-seed 2147483648.d0))
  (defun reset-urand ()
    (setf urand-seed 1)
    (values)))

(defun make-and-init-vector (element-type size &key init-offset)
  "Make a vector of the given element type and size. If init-offset is given,
   it is assumed to be a valid number, with which the vector is initialised;
   each element is set to a unique, predetermined value. See also test.c in
   GSL's fft directory."
  (let ((vec (grid:make-foreign-array element-type :dimensions (list size))))
    (when init-offset
      (loop for i from 0 below size
            do
            (setf (grid:aref vec i)
                  (if (subtypep element-type 'complex)
                    (coerce (complex (+ (* 2 i) init-offset)
                                     (+ (* 2 i) init-offset 1))
                            element-type)
                    (coerce (+ i init-offset) element-type))))
    vec)))

;; (make-urand-vector '(complex double-float) 5)
(defun make-urand-vector (element-type dimension &key (stride 1) init-offset)
  "Make a vector with random elements."
  (let ((vec (make-and-init-vector `(complex ,(grid:component-float-type element-type))
                                   (* stride dimension) :init-offset init-offset)))
    (loop for i from 0 below (* stride dimension) by stride
       do
       (setf (grid:aref vec i)
	     (if (subtypep element-type 'complex)
		 (coerce (complex (urand) (urand)) element-type)
		 (complex (coerce (urand) element-type)))))
    vec))

(defun realpart-vector (complex-vector &key (stride 1) init-offset)
  "The real vector consisting of the real part of the complex vector."
  (let ((real-vector
          (make-and-init-vector
            (grid:component-float-type (grid:element-type complex-vector))
            (size complex-vector)
            :init-offset init-offset)))
    (loop for i below (size complex-vector) by stride
          do
          (setf (grid:aref real-vector i)
                (realpart (grid:aref complex-vector i))))
    real-vector))

(defun copy-with-stride (vector &key (stride 1) init-offset)
  "Copy a vector and initialize it."
  (let ((vec
	 (make-and-init-vector
	  (grid:element-type vector)
	  (size vector)
	  :init-offset init-offset)))
    (loop for i below (size vector) by stride
          do
       (setf (grid:aref vec i) (grid:aref vector i)))
    vec))

(defun size-vector-scalar (vector &key (stride 1))
  "Return the size of a vector while taking the stride into account."
  (coerce (floor (size vector) stride)
	  (if (subtypep (grid:element-type vector) 'complex)
	      (grid:element-type vector)
	      'double-float)))

#+nil
(defun vector/length (vector stride)
  (elt/ vector (size-vector-scalar vector :stride stride)))

(defun vector/length (vector &key (stride 1))
  (let ((element-type (grid:element-type vector)))
    (loop with length = (size-vector-scalar vector :stride stride)
          for i from 0 below (size vector) by stride
          do
          (setf (grid:aref vector i) (coerce (/ (grid:aref vector i) length) element-type)))
  vector))

(defun test-real-fft-noise (vector &key (stride 1) non-radix-2)
  "Test forward and inverse FFT for a real vector, and return both results in unpacked form."
  (let* ((forward
	  (forward-fourier-transform (realpart-vector vector :stride stride :init-offset 0)
                                     :stride stride :non-radix-2 non-radix-2))
         (inverse
	  (forward-fourier-transform (copy forward) :half-complex t :stride stride
				     :non-radix-2 non-radix-2)))
    (values (unpack forward :unpack-type 'complex :stride stride)
            (unpack (vector/length inverse :stride stride) :stride stride))))

(defun test-complex-fft-noise (vector &key (stride 1) non-radix-2)
  "Test forward, inverse and backward FFT for a complex vector and return all three results."
  (let ((forward
          (forward-fourier-transform
            (copy-with-stride vector :stride stride :init-offset 2000)
            :stride stride :non-radix-2 non-radix-2)))
    (values forward
	    (inverse-fourier-transform (copy-with-stride forward :stride stride :init-offset 0)
                                       :stride stride :non-radix-2 non-radix-2)
            ;; in backward-fourier-transform, we could use copy instead of ;;
            ;; copy-with-stride
            (backward-fourier-transform (copy-with-stride forward :init-offset
                                                          2000)
                                        :stride stride :non-radix-2 non-radix-2))))

(defun test-fft-noise (element-type size &key (stride 1) non-radix-2)
  "A test of real forward and complex forward, reverse, and inverse FFT
   on random noise.  Returns the result of the DFT forward Fourier transformation
   and the forward FFT, which should be the same, and the original vector and the
   inverse FFT, which should also be the same.  In addition,
   the backward Fourier transform is returned for complex vectors, which
   should be the same as the last two."
  (let* ((random-vector (make-urand-vector element-type size :stride stride
                                           :init-offset 1000))
         (dft-random-vector
           (forward-discrete-fourier-transform random-vector :stride stride
                                               :result (make-and-init-vector
                                                         `(complex ,(grid:component-float-type element-type))
                                                         (* size stride)
                                                         :init-offset 3000))))
    (if (subtypep element-type 'complex)
	(multiple-value-bind (forward inverse backward)
	    (test-complex-fft-noise random-vector :stride stride
				    :non-radix-2 non-radix-2)
	  (values
	   dft-random-vector		; DFT forward result for reference
	   forward			; FFT forward result; should check
	   random-vector		; The original vector for reference
	   inverse			; The inverse FFT applied to the forward result
	   (vector/length backward :stride stride)))
	(multiple-value-bind (forward inverse)
	    (test-real-fft-noise random-vector :stride stride
				 :non-radix-2 non-radix-2)
	  (values dft-random-vector forward random-vector inverse)))))

;; (test-fft-noise '(complex double-float) 2 :stride 2)
;; (test-fft-noise '(complex double-float) 10 :stride 2)
;; (test-fft-noise 'double-float 10 :stride 1)
;; (test-fft-noise '(complex double-float) 10 :stride 1)

;;;;****************************************************************************
;;;; Constructing tests
;;;;****************************************************************************

#+(or)
(defun make-real-noise-fft-test (size stride)
  "Make a test of a random real-vector FFT." 
  (reset-urand)
  (let* ((randvec (make-urand-vector 'double-float size :stride stride))
	 (dft (forward-discrete-fourier-transform (copy randvec) :stride stride)))
    (list
     ;; Forward
     (make-test
      `(unpack (forward-fourier-transform ; computed FFT
		,(make-load-form (realpart-vector (copy randvec)))
		:stride ,stride)
	       :unpack-type 'complex :stride ,stride)
      dft)
     ;; Inverse
     (make-test
      `(unpack
	(vector/length
	 (forward-fourier-transform
	  ,(make-load-form (realpart-vector dft))
	  :half-complex t :stride ,stride)
	 ,stride)
	:stride ,stride)
      randvec))))

;; loop size from 1 to 99, stride from 1, 2, 3
#+(or)
(defun generate-fft-tests (&optional (size-max 99) (stride-max 3))
  (append
   '(lisp-unit:define-test fft-noise)
   (loop for size from 1 to size-max
      append
      (loop for stride from 1 to stride-max
	 append (make-real-noise-fft-test size stride)))))
