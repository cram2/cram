;; Combinations
;; Liam Healy, Sun Mar 26 2006 - 11:51
;; Time-stamp: <2010-07-16 17:14:03EDT combination.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2010 Liam M. Healy
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

;;; /usr/include/gsl/gsl_combination.h

;;;;****************************************************************************
;;;; Combination structure and CL object
;;;;****************************************************************************

(defclass combination 
    (#+int64 grid:vector-unsigned-byte-64 #+int32 grid:vector-unsigned-byte-32)
  ()
  (:documentation "GSL combinations."))

(defmethod initialize-instance :after
    ((object combination) &key range dimensions &allow-other-keys)
  (let ((mptr (cffi:foreign-alloc 'gsl-combination-c)))
    (setf (grid:metadata-slot object 'mpointer)
	  mptr
	  (cffi:foreign-slot-value mptr 'gsl-combination-c 'data)
	  (foreign-pointer object)
	  (cffi:foreign-slot-value mptr 'gsl-combination-c 'range)
	  range
	  (cffi:foreign-slot-value mptr 'gsl-combination-c 'size)
	  (first dimensions))
    (tg:finalize object (lambda () (cffi:foreign-free mptr)))))

(export 'make-combination)

(defun make-combination (n &optional k (initialize t))
  "Make the object representing a combination of k things from a set of n.
   If initialize is T, initialize as the first k values (init-first).
   If n is a combination, make a new combination with the same
   specification.  If initialize is also T, copy it."
  (let ((comb
	 (if (typep n 'combination)
	     (make-instance
	      'combination
	      :element-type '(unsigned-byte #+int64 64 #+int32 32)
	      :range (combination-range n) :dimensions (dimensions k))
	     (make-instance
	      'combination
	      :element-type '(unsigned-byte #+int64 64 #+int32 32)
	      :range n :dimensions (list k)))))
    (when initialize
      (if (typep n 'combination)
	  (error "not available yet")	; (copy comb n)
	  (init-first comb)))
    comb))

(defmethod print-object ((object combination) stream)
  (print-unreadable-object (object stream :type t)
    (format stream "range ~d: " (combination-range object))
    (princ (grid:contents object) stream)))

;;;;****************************************************************************
;;;; Setting values
;;;;****************************************************************************

(defmfun init-first (combination)
  "gsl_combination_init_first"
  (((mpointer combination) :pointer))
  :c-return :void
  :outputs (combination)
  :documentation			; FDL
  "Initialize the combination c to the lexicographically
      first combination, i.e.  (0,1,2,...,k-1).")

(defmfun init-last (combination)
  "gsl_combination_init_last"
  (((mpointer combination) :pointer))
  :c-return :void
  :outputs (combination)
  :documentation			; FDL
  "Initialize the combination c to the lexicographically
   last combination, i.e. (n-k,n-k+1,...,n-1).")

(defmfun comb-copy (source destination)
  "gsl_combination_memcpy"
  (((mpointer destination) :pointer)
   ((mpointer source) :pointer))
  :inputs (source)
  :outputs (destination)
  :return (destination)
  :export nil
  :index grid:copy
  :documentation			; FDL
  "Copy the elements of the combination source into the
  combination destination.  The two combinations must have the same size.")

(defmethod grid:copy
    ((source combination) &key grid-type destination &allow-other-keys)
  (if grid-type
      (call-next-method)
      (comb-copy
       source
       (or destination
	   (make-combination (combination-range source) (size source))))))

;;;;****************************************************************************
;;;; Combination properties
;;;;****************************************************************************

(defmfun combination-range (c)
  "gsl_combination_n"
  (((mpointer c) :pointer))
  :c-return sizet
  :inputs (c)
  :documentation			; FDL
  "The range (n), or maximum possible value (n in the (n k) notation)
   of the combination c.")

(defmfun size ((c combination))
  "gsl_combination_k"
  (((mpointer c) :pointer))
  :definition :method
  :c-return sizet
  :documentation			; FDL
  "The number of elements (k) in the combination c.")

#|
;;; Unnecessary, gsl-array serves this function.
(defmfun combination-data (c)
  "gsl_combination_data"
  (((mpointer c) :pointer))
  :c-return :pointer
  :documentation			; FDL
  "A pointer to the array of elements in the combination.")
|#

(defmfun validp ((combination combination))
  "gsl_combination_valid"
  (((mpointer combination) :pointer))
  :definition :method 
  :c-return :success-failure
  :documentation			; FDL
  "Check that the combination is valid.  The k
   elements should lie in the range 0 to n-1, with each
   value occurring once at most and in increasing order.")

;;;;****************************************************************************
;;;; Combination functions
;;;;****************************************************************************

(defmfun combination-next (c)
  "gsl_combination_next" (((mpointer c) :pointer))
  :c-return (crtn :int)
  :inputs (c)
  :outputs (c)
  :return ((when (success-failure crtn) c))
  :documentation			; FDL
  "Advance the combination c to the next combination in lexicographic
   order and return c.  If no further combinations are available
   it returns NIL.  Starting with the first combination and repeatedly
   applying this function will iterate through all possible
   combinations of a given order.")

(defmfun combination-previous (c)
  "gsl_combination_prev"
  (((mpointer c) :pointer))
  :c-return (crtn :int)
  :inputs (c)
  :outputs (c)
  :return ((when (success-failure crtn) c))
  :documentation			; FDL
  "Step backwards from the combination c to the previous combination
   in lexicographic order, returning c.  If no previous combination is
   available it returns NIL with c unmodified.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test combination
 (let ((comb (make-combination 4 2)))	; combination-range
   (combination-range comb))
 (let ((comb (make-combination 4 2)))	; size
   (size comb))
 (let ((comb (make-combination 4 2)))	; init-first, combination-next
   (init-first comb)
   (loop collect (grid:contents comb)
	 while (combination-next comb)))
 (let ((comb (make-combination 4 2)))  ; init-last, combination-previous
   (init-last comb)
   (loop collect (grid:contents comb)
	 while (combination-previous comb)))
 (loop for i from 0 to 4		; combination-next
       append
       (let ((comb (make-combination 4 i)))
	 (init-first comb)
	 (loop collect (grid:contents comb)
	       while (combination-next comb)))))
