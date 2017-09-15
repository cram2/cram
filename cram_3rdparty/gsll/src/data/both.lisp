;; Functions for both vectors and matrices.
;; Liam Healy 2008-04-26 20:48:44EDT both.lisp
;; Time-stamp: <2012-01-13 12:01:36EST both.lisp>
;;
;; Copyright 2008, 2009, 2010, 2011 Liam M. Healy
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
;;;; Administrative (internal use)
;;;;****************************************************************************

(defmfun alloc-from-block ((object vector) blockptr)
  ("gsl_" :category :type "_alloc_from_block")
  ((blockptr :pointer)
   (0 :sizet)				; offset
   ((size object) :sizet)		; number of elements
   (1 :sizet))				; stride
  :definition :generic
  :c-return :pointer
  :export nil
  :documentation "Allocate memory for the GSL struct given a block pointer.")

(defmfun alloc-from-block ((object grid:matrix) blockptr)
  ("gsl_" :category :type "_alloc_from_block")
  ((blockptr :pointer)
   (0 :sizet)				; offset
   ((first (grid:dimensions object)) :sizet)	; number of rows
   ((second (grid:dimensions object)) :sizet)	; number of columns
   ((second (grid:dimensions object)) :sizet))	; "tda" = number of columns for now
  :definition :methods
  :c-return :pointer
  :export nil)

;;;;****************************************************************************
;;;; Bulk operations
;;;;****************************************************************************

(defmfun set-all ((object both) value)
  ("gsl_" :category :type "_set_all")
  (((mpointer object) :pointer) (value :element-c-type))
  :definition :generic
  :element-types #+fsbv t #-fsbv :no-complex
  :outputs (object)
  :c-return :void
  :documentation "Set all elements to the value.")

(defmfun set-zero ((object both))
  ("gsl_"  :category :type "_set_zero")
  (((mpointer object) :pointer))
  :definition :generic
  :inputs (object)
  :outputs (object)
  :c-return :void
  :documentation "Set all elements to 0.")

(defmfun swap ((a both) (b both))
  ("gsl_" :category :type "_swap")
  (((mpointer a) :pointer) ((mpointer b) :pointer))
  :definition :generic
  :inputs (a b)
  :outputs (a b)
  :documentation			; FDL
  "Exchange the elements of a and b
   by copying.  The two must have the same dimensions.")

;;;;****************************************************************************
;;;; Array elements; used in callbacks scalarsp=T only
;;;;****************************************************************************
;;; Normal foreign array access is with grid:aref, but in order to
;;; avoid the overhead of instantiating a foreign-array object to
;;; access components, we use these macros which expand to gsl_*_get
;;; and gsl_*_set.

;;; It's unlikely users will need maref directly, because when using
;;; the GSL library, we use the grid functions to get elements.  The
;;; main use in the GSL library is user-defined callbacks (e.g. in
;;; solve-minimize-fit) when scalarsp = T is specified; in that case
;;; the generated wrapper function includes maref.  However, if
;;; another foreign library which used the GSL library provided an
;;; mpointer, the maref macro saves the time of creating an object
;;; around it.
(export 'maref)

(defun access-value-int (mpointer class-name value indices)
  "Create a form to access the GSL array value from the mpointer.  If value is not nil,
   set the value; otherwise, get the value."
  ;; (access-value-int 'ptr 'grid:vector-unsigned-byte-16 45 '(3))
  ;; (FOREIGN-FUNCALL "gsl_vector_ushort_set" :POINTER PTR :SIZET 3 :UNSIGNED-SHORT 45 :VOID)
  ;; (access-value-int 'ptr 'grid:matrix-unsigned-byte-16 nil '(45 3))
  ;; (FOREIGN-FUNCALL "gsl_matrix_ushort_get" :POINTER PTR :SIZET 45 :SIZET 3 :UNSIGNED-SHORT)
  (let ((element-type (grid::farray-element-type class-name))
	(matrixp (subtypep class-name 'grid:matrix)))
    (unless (or (and matrixp (eql 2 (length indices)))
		(and (not matrixp) (eql 1 (length indices))))
      (error "The number of indices, ~a, is not correct for an array of class ~a"
	     (length indices)
	     class-name))
    `(cffi:foreign-funcall
      ,(actual-gsl-function-name
	`("gsl_" :category :type ,(if value "_set" "_get"))
	(if matrixp 'grid:matrix 'vector)
	element-type)
      :pointer ,mpointer
      :sizet ,(first indices)
      ,@(when matrixp (list ':sizet (second indices)))
      ,(grid:cl-cffi element-type)
      ,@(when value `(,value :void)))))

(defmacro maref (mpointer class-name &rest indices)
  "Get or set (setf maref) the array element from the GSL mpointer.
   The class-name is the specific subclass name of
   grid:foreign-array."
  (access-value-int mpointer class-name nil indices))

(defmacro set-maref (mpointer class-name &rest indices-value)
  (alexandria:once-only ((value (alexandria:lastcar indices-value)))
    `(progn
       ,(access-value-int
	 mpointer
	 class-name
	 value
	 (butlast indices-value))
       ,value)))

(defsetf maref set-maref)

;;; (maref ptr grid:vector-double-float 3)
;;; (setf (maref ptr grid:vector-double-float 3) 45.0d0)

;;;;****************************************************************************
;;;; Elementwise arithmetic operations overwriting an array
;;;;****************************************************************************

;;; These are overwriting elementwise arithmetic operations.  Where
;;; there are two array arguments, the operation acts on both arrays
;;; on corresponding elements.  The result is placed in the first
;;; array, overwriting the original contents.

;;; Errors in GSL:
;;; 1) complex operations in older versions of GSL
;;; https://savannah.gnu.org/bugs/index.php?22478
;;; Fixed in 1.12, can change the :no-complex spec.

(defmfun elt+ ((a both) (b both))
  ("gsl_" :category :type "_add")
  (((mpointer a) :pointer) ((mpointer b) :pointer))
  :definition :generic
  :element-types #.(if (have-at-least-gsl-version '(1 12)) t :no-complex)
  :inputs (a b)
  :outputs (a)
  :return (a)
  :documentation			; FDL
  "Add the elements of b to the elements of vector a
   The two must have the same dimensions.")

(defmfun elt+ ((a both) (x float))
  ("gsl_" :category :type "_add_constant")
  (((mpointer a) :pointer) (x :double))
  :definition :methods
  :element-types :no-complex
  :inputs (a)
  :outputs (a)
  :return (a)
  :documentation			; FDL
  "Add the scalar double-float x to all the elements of array a.")

#+fsbv
(defmfun elt+ ((a both) (x complex))
  ("gsl_" :category :type "_add_constant")
  (((mpointer a) :pointer) (x :element-c-type))
  :definition :methods
  :element-types :complex
  :inputs (a)
  :outputs (a)
  :return (a)
  :gsl-version (1 12)
  :documentation			; FDL
  "Add the scalar complex x to all the elements of array a.")

(defmethod elt+ ((x float) (a grid:foreign-array))
  (elt+ a x))
  
(defmfun elt- ((a both) (b both))
  ("gsl_" :category :type "_sub")
  (((mpointer a) :pointer) ((mpointer b) :pointer))
  :definition :generic
  :element-types #.(if (have-at-least-gsl-version '(1 12)) t :no-complex)
  :inputs (a b)
  :outputs (a)
  :return (a)
  :documentation			; FDL
  "Subtract the elements of b from the elements of a.
   The two must have the same dimensions.")

(defmethod elt- ((a grid:foreign-array) (x float))
  (elt+ a (- x)))

(defmfun elt* ((a vector) (b vector))
  ("gsl_" :category :type "_mul")
  (((mpointer a) :pointer) ((mpointer b) :pointer))
  :definition :generic
  :element-types #.(if (have-at-least-gsl-version '(1 12)) t :no-complex)
  :inputs (a b)
  :outputs (a)
  :return (a)
  :documentation			; FDL
  "Multiply the elements of a by the elements of b.
   The two must have the same dimensions.")

(defmfun elt* ((a grid:matrix) (b grid:matrix))
  ("gsl_" :category :type "_mul_elements")
  (((mpointer a) :pointer) ((mpointer b) :pointer))
  :definition :methods
  :element-types :no-complex
  :inputs (a b)
  :outputs (a)
  :return (a))

(defmfun elt/ ((a vector) (b vector))
  ("gsl_" :category :type "_div")
  (((mpointer a) :pointer) ((mpointer b) :pointer))
  :definition :generic
  :element-types #.(if (have-at-least-gsl-version '(1 12)) t :no-complex)
  :inputs (a b)
  :outputs (a)
  :return (a)
  :documentation			; FDL
  "Divide the elements of a by the elements of b.
   The two must have the same dimensions.")

(defmfun elt/ ((a grid:matrix) (b grid:matrix))
  ("gsl_" :category :type "_div_elements")
  (((mpointer a) :pointer) ((mpointer b) :pointer))
  :definition :methods
  :element-types :no-complex
  :inputs (a b)
  :outputs (a)
  :return (a))

(defmethod elt/ ((a grid:foreign-array) (x number))
  (elt* a (/ x)))

(defmfun elt* ((a both) (x float))
  ("gsl_" :category :type "_scale")
  (((mpointer a) :pointer) (x :double))
  :definition :methods
  :element-types :no-complex
  :inputs (a)
  :outputs (a)
  :return (a)
  :documentation			; FDL
  "Multiply the elements of a by the scalar double-float factor x.")

#+fsbv
(defmfun elt* ((a both) (x complex))
  ("gsl_" :category :type "_scale")
  (((mpointer a) :pointer) (x :element-c-type))
  :definition :methods
  :element-types :complex
  :inputs (a)
  :outputs (a)
  :return (a)
  :gsl-version (1 12)
  :documentation			; FDL
  "Multiply the elements of a by the scalar complex factor x.")

(defmethod elt* ((x float) (a grid:foreign-array))
  (elt* a x))

;;;;****************************************************************************
;;;; Maximum and minimum elements
;;;;****************************************************************************

(defmfun mmax ((a both))
  ("gsl_" :category :type "_max")
  (((mpointer a) :pointer))
  :definition :generic
  :element-types :no-complex
  :inputs (a)
  :c-return :element-c-type
  :documentation			; FDL
  "The maximum value in a.")

(defmfun mmin ((a both))
  ("gsl_" :category :type "_min")
  (((mpointer a) :pointer))
  :definition :generic
  :element-types :no-complex
  :inputs (a)
  :c-return :element-c-type
  :documentation			; FDL
  "The minimum value in a.")

(defmfun minmax ((a both))
  ("gsl_" :category :type "_minmax")
  (((mpointer a) :pointer)
   (min (:pointer :element-c-type))
   (max (:pointer :element-c-type)))
  :definition :generic
  :element-types :no-complex
  :inputs (a)
  :c-return :void
  :documentation			; FDL
  "The minimum and maximum values in a.")

(defmfun min-index ((a vector))
  ("gsl_" :category :type "_min_index")
  (((mpointer a) :pointer))
  :definition :generic
  :element-types :no-complex
  :inputs (a)
  :c-return :sizet
  :documentation			; FDL
  "The index of the minimum value in a.  When there are several
  equal minimum elements, then the lowest index is returned.")

(defmfun min-index ((a grid:matrix))
  ("gsl_" :category :type "_min_index")
  (((mpointer a) :pointer) (imin (:pointer :sizet)) (jmin (:pointer :sizet)))
  :definition :methods
  :element-types :no-complex
  :inputs (a)
  :c-return :void)

(defmfun max-index ((a vector))
  ("gsl_" :category :type "_max_index")
  (((mpointer a) :pointer))
  :definition :generic
  :element-types :no-complex
  :inputs (a)
  :c-return :sizet
  :documentation			; FDL
  "The index of the maximum value in a.  When there are several
  equal maximum elements, then the lowest index is returned.")

(defmfun max-index ((a grid:matrix))
  ("gsl_" :category :type "_max_index")
  (((mpointer a) :pointer) (imin (:pointer :sizet)) (jmin (:pointer :sizet)))
  :definition :methods
  :element-types :no-complex
  :inputs (a)
  :c-return :void)

(defmfun minmax-index ((a vector))
  ("gsl_" :category :type "_minmax_index")
  (((mpointer a) :pointer) (imin (:pointer :sizet)) (jmin (:pointer :sizet)))
  :definition :generic
  :element-types :no-complex
  :inputs (a)
  :c-return :void
  :documentation			; FDL
  "The indices of the minimum and maximum values in a.
  When there are several equal minimum elements then the lowest index is
  returned.  Returned indices are minimum, maximum; for matrices
  imin, jmin, imax, jmax.")

(defmfun minmax-index ((a grid:matrix))
  ("gsl_" :category :type "_minmax_index")
  (((mpointer a) :pointer)
   (imin (:pointer :sizet)) (jmin (:pointer :sizet))
   (imax (:pointer :sizet)) (jmax (:pointer :sizet)))
  :definition :methods
  :element-types :no-complex
  :inputs (a)
  :c-return :void)

;;;;****************************************************************************
;;;; Properties
;;;;****************************************************************************

(defmfun mzerop ((a both))
  ("gsl_" :category :type "_isnull")
  (((mpointer a) :pointer))
  :definition :generic
  :inputs (a)
  :c-return :boolean
  :documentation			; FDL
  "All elements of a are zero.")

(defmfun mplusp ((a both))
  ("gsl_" :category :type "_ispos")
  (((mpointer a) :pointer))
  :definition :generic
  :gsl-version (1 9)
  :inputs (a)
  :c-return :boolean
  :documentation			; FDL
  "All elements of a are positive.")

(defmfun mminusp ((a both))
  ("gsl_" :category :type "_isneg")
  (((mpointer a) :pointer))
  :definition :generic
  :gsl-version (1 9)
  :inputs (a)
  :c-return :boolean
  :documentation			; FDL
  "All elements of a are negative.")

(defmfun non-negative-p ((a both))
  ("gsl_" :category :type "_isnonneg")
  (((mpointer a) :pointer))
  :definition :generic
  :gsl-version (1 10)
  :inputs (a)
  :c-return :boolean
  :documentation			; FDL
  "All elements of a are non-negative.")
