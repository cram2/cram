;; Define examples.
;; Liam Healy 2008-09-07 21:00:48EDT generate-tests.lisp
;; Time-stamp: <2010-06-29 22:15:23EDT generate-examples.lisp>
;;
;; Copyright 2008, 2009 Liam M. Healy
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

;;; Define examples that can be displayed by users with the function
;;; #'examples and can be used to generate regression (unit) tests.
;;; The latter has been done in the gsll-tests system.

(in-package :gsl)

(export 'examples)

;;;;****************************************************************************
;;;; Defining examples
;;;;****************************************************************************

(defparameter *all-generated-tests* nil) 

(defmacro save-test (name &rest forms)
  "Save the test with the given name."
  `(setf
    (getf *all-generated-tests* ',name)
    (remove-duplicates
     (append
      ',forms
      (getf *all-generated-tests* ',name))
     :test #'equal)))

;;; This is needed for debugging, to remove the previous definitions.
(defun delete-test-definition (name)
  (remf *all-generated-tests* name))

(defun examples (&optional name)
  "If no argument is supplied, list the names of the example categories.
   If a category name is given as the argument, give the list of examples
   in that category."
  (if name 
      (getf *all-generated-tests* name)
      (loop for pr on *all-generated-tests* by #'cddr
	 collect (first pr))))

;;;;****************************************************************************
;;;; Data pool 
;;;;****************************************************************************

;; Signed
;;(loop repeat 100 collect (* (if (zerop (random 2)) -1 1) (random 128)))
;; Unsigned
;;(loop repeat 100 collect (random 128))
;;(loop repeat 100 collect (random 128.0d0))
;; Make short-decimal floats for floats?

;;; (loop repeat 50 collect (coerce (- (/ (random 10000) 100) 50) 'double-float))
(defparameter *double-float-pool*
  '(-34.5d0 8.24d0 3.29d0 -8.93d0 34.12d0 -6.15d0 49.27d0 -13.49d0 32.5d0
    42.73d0 -17.24d0 43.31d0 -16.12d0 -8.25d0 21.44d0 -49.08d0 -39.66d0
    -49.46d0 19.68d0 -5.55d0 -8.82d0 25.37d0 -30.58d0 31.67d0 29.36d0
    -33.24d0 -27.03d0 -41.67d0 42.0d0 -20.81d0 37.93d0 -30.02d0 11.41d0
    22.0d0 33.43d0 42.97d0 -36.7d0 -9.69d0 10.18d0 -47.96d0 37.42d0 -42.55d0
    40.77d0 18.19d0 -43.66d0 -9.3d0 -29.79d0 -38.88d0 -45.6d0 -31.91d0)
  "A sequence of random double floats ranging between -100.0d0 and +100.0d0.")

;;; (loop repeat 50 collect (random 256))
(defparameter *unsigned-byte-pool*
  '(67 44 189 116 163 140 161 215 98 28 10 19 28 178 217 36 109 222 88 97 167
    135 96 198 37 110 12 175 184 62 40 27 146 128 18 237 184 234 24 43 79 49
    156 158 157 167 157 34 219 249)
  "A sequence of random integers ranging between 0 and 255.")

;;; (loop repeat 50 collect (* (if (zerop (random 2)) -1 1) (random 128)))
(defparameter *signed-byte-pool*
  '(-64 -68 71 -91 52 -10 73 -5 123 32 28 30 37 -73 -8 -15 -22 68 -47 -81 -68
    125 -15 -53 -17 -114 24 -60 32 106 -3 37 -1 97 -96 -12 -20 -61 18 108 61
    -82 75 -30 -71 44 48 88 121 106)
  "A sequence of random integers ranging between -255 and 255.")

(defun make-list-from-pool (type length &optional (starting 0))
  "Make a list for :initial-contents of the specified element type and
   length using the pool data for the type and starting at the
   specified point in the pool."
  (mapcar
   (lambda (num) (coerce num (grid:component-float-type type)))
   (subseq
    (cond ((subtypep type 'unsigned-byte) *unsigned-byte-pool*)
	  ((subtypep type 'signed-byte) *signed-byte-pool*)
	  ((subtypep type 'float) *double-float-pool*)
	  ((subtypep type 'complex) *double-float-pool*))
    starting
    (+ starting (if (subtypep type 'complex) (* 2 length) length)))))


;;;;****************************************************************************
;;;; Generate forms for all array types
;;;;****************************************************************************

(defun array-default (spec &optional no-init)
  "Make an array of the current type and initialize from the pool."
  (declare (special default-element-type starting-element))
  (let ((matrixp (listp spec)))
    (if no-init
	;; No initial values, just dimension
	`(grid:make-foreign-array ',default-element-type :dimensions ',spec)
	;; Initial contents
	`(grid:make-foreign-array
	  ',default-element-type
	  :initial-contents
	  ',(if matrixp
		(loop repeat (first spec)
		   collect
		   (make-list-from-pool
		    default-element-type (second spec) starting-element)
		   do
		   (incf starting-element (second spec)))
		(prog1 
		    (make-list-from-pool
		     default-element-type spec starting-element)
		  (incf starting-element spec)))))))

(defun scalar-default (&optional float-type)
  "Make a scalar of the current type from the pool.  For complex
   types, setting float-type will select a real of the corresponding
   component float type."
  (declare (special default-element-type starting-element))
  (prog1
      (if (subtypep default-element-type 'complex)
	  (if float-type
	      (first
	       (make-list-from-pool
		(grid:component-float-type default-element-type) 1 starting-element))
	      (apply
	       #'complex
	       (make-list-from-pool default-element-type 1 starting-element)))
	  (first
	   (make-list-from-pool default-element-type 1 starting-element)))
    (incf starting-element)))

(defun stupid-code-walk-eval-some (form eval-list)
  "Walk the form and if the first symbol of the list is a member of
   eval-list, evaluate it and use the result.  Otherwise use the result
   as-is."
  (if (atom form)
      form
      (if (member (first form) eval-list)
	  (eval form)
	  (mapcar (lambda (se)
		    (stupid-code-walk-eval-some se eval-list))
		  form))))

(defun generate-all-array-tests-body (element-types test)
  (loop for det in (grid:element-types element-types)
     collect
     (let ((default-element-type det))
       (declare (special default-element-type starting-element))
       (setf starting-element 0)
       (stupid-code-walk-eval-some
	(subst det 'default-element-type test)
	'(array-default scalar-default)))))

(defmacro generate-all-array-tests (name element-types test)
  `(save-test ,name ,@(generate-all-array-tests-body element-types test)))

