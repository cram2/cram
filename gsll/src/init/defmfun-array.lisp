;; Helpers for defining GSL functions on arrays
;; Liam Healy 2009-01-07 22:01:16EST defmfun-array.lisp
;; Time-stamp: <2011-01-29 21:40:31EST defmfun-array.lisp>
;;
;; Copyright 2009, 2011 Liam M. Healy
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
;;;; A defgeneric with methods, or the methods alone, for vectors or matrices
;;;;****************************************************************************

(defun expand-defmfun-arrays
    (defn name arglist gsl-name c-arguments categories key-args)
  (setf categories
	(remove-if-not
	 (lambda (c) (member c '(both grid:matrix vector)))
	 categories))
  (if (member 'both categories)
      (progn 
	(when (> (length categories) 1)
	  ;; Specify 'both alone
	  (error "Internal: mixed both and actual category."))
	;; Generate forms for matrix and vector
	(append
	 (generate-methods
	  defn 'vector
	  name arglist gsl-name
	  (actual-array-c-type
	   'vector c-arguments
	   (optional-args-to-switch-gsl-functions arglist gsl-name))
	  (copy-list key-args) 'vector)
	 (generate-methods
	  defn 'grid:matrix
	  name arglist gsl-name
	  (actual-array-c-type
	   'grid:matrix c-arguments
	   (optional-args-to-switch-gsl-functions arglist gsl-name))
	  (copy-list key-args) 'grid:matrix)))
      ;; Generate forms for one category
      (generate-methods
       defn (first categories)
       name arglist gsl-name c-arguments key-args)))

(defun expand-defmfun-generic (name arglist gsl-name c-arguments key-args)
  "Define a generic function and methods for all kinds of arrays."
  ;; Need to scan the arglist for categories.
  ;; Can be mixed, unless it says 'both, in which case it can only be both.
  (with-defmfun-key-args key-args
    (multiple-value-bind (noclass-arglist categories)
	(arglist-plain-and-categories arglist)
      `(defgeneric ,name
	   ;; Remove &aux from the arglist, it is not allowed in
	   ;; defgeneric arglists.
	   ,(subseq noclass-arglist 0 (position '&aux noclass-arglist))
	 (:documentation ,documentation)
	 ,@(expand-defmfun-arrays
	    :method name arglist gsl-name c-arguments categories key-args)))))

(defun expand-defmfun-defmethods (name arglist gsl-name c-arguments key-args)
  "Define methods for all kinds of arrays."
  (let ((categories (nth-value 1 (arglist-plain-and-categories arglist))))
    (expand-defmfun-arrays
     'cl:defmethod name arglist gsl-name c-arguments categories key-args)))

(defun generate-methods
    (defn category name arglist gsl-name c-arguments key-args
     &optional replace-both)
  "Create all the methods for a generic function."
  ;; Methods may have &optional in two ways: the optional argument(s)
  ;; are defaulted if not supplied and a single GSL function called,
  ;; or the presence/absence of optional arguments switches between two
  ;; GSL functions.
  (unless category (error "category is nil"))
  (with-defmfun-key-args key-args
    (mapcar (lambda (eltype)
	      (flet ((actual-gfn (gslname)
		       (let ((gsl-function-name
			      (actual-gsl-function-name
			       gslname category eltype)))
			 (push gsl-function-name indexed-functions)
			 gsl-function-name)))
		(remf key-args :documentation)
		(when (eq c-return :element-c-type)
		  (setf (getf key-args :c-return) (grid:cl-cffi eltype)))
		(when (eq c-return :component-float-type)
		  (setf (getf key-args :c-return) (grid:component-type eltype)))
		(if (optional-args-to-switch-gsl-functions arglist gsl-name)
		    ;; The methods have optional argument(s) and
		    ;; multiple GSL functions for presence/absence of
		    ;; options
		    (complete-definition
		     defn
		     (if (eq defn :method) (list nil name) name)
		     (actual-class-arglist arglist eltype c-arguments replace-both)
		     (mapcar #'actual-gfn gsl-name)
		     (mapcar (lambda (args)
			       (actual-element-c-type eltype args))
			     c-arguments)
		     key-args
		     'body-optional-arg)
		    (complete-definition
		     defn
		     (if (eq defn :method) (list nil name) name)
		     (actual-class-arglist arglist eltype c-arguments replace-both)
		     (actual-gfn gsl-name)
		     (actual-element-c-type eltype c-arguments)
		     key-args))))
	    (grid:element-types element-types))))

(defun actual-gsl-function-name (base-name category type)
  "Create the GSL or BLAS function name for data from the base name
   and the CL type."
  ;; (actual-gsl-function-name
  ;;  '("gsl_" :category :type "_swap") 'vector '(unsigned-byte 16))
  ;; "gsl_vector_ushort_swap"
  (if (listp base-name)
      (if (symbolp (first base-name))
	  ;; An explicit listing of types with function names
	  (getf base-name (grid:cl-single type :gsl))
	  (let ((blas (search "blas" (first base-name) :test 'string-equal)))
	    (apply #'concatenate 'string
		   (substitute
		    (if (subtypep type 'complex) "_complex" "") :complex
		    (substitute
		     (if (subtypep type 'complex) "u" "") :suffix
		     (substitute
		      (cl-gsl type (not blas) blas) :type
		      (substitute
		       (if (subtypep type 'complex)
			   (cl-gsl (grid:component-float-type type) nil blas)
			   "") :component-float-type
		       (substitute
			(string-downcase (symbol-name category)) :category
			base-name))))))))))

(defun actual-array-class (category element-type &optional replace-both)
  "From the category ('vector, 'grid:matrix, or 'both) and element type,
   find the class name."
  (case category
    (:element-type (grid:number-class element-type))
    (:component-float-type
       (grid:number-class (grid:component-float-type element-type)))
    ((vector grid:matrix both)
       (grid:data-class-name
	(if (and (eq category 'both) replace-both)
	    replace-both
	    category)
	element-type))
    ;; an actual not-to-be-replaced class name
    (otherwise category)))

(defun actual-class-arglist
    (arglist element-type c-arguments &optional replace-both)
  "Replace the prototype arglist with an actual arglist."
  (loop for arg in arglist
     with replacing = t
     with carg-actual = (actual-element-c-type element-type c-arguments)
     do
     (when (and replacing (member arg *defmfun-llk*))
       (setf replacing nil))
     collect
     (if (and replacing (listp arg))
	 (list (first arg)
	       (if (eql-specializer arg)
		   ;; eql specializer on class name
		   `(eql
		     ',(actual-array-class
			(eql-specializer arg) element-type replace-both))
		   (actual-array-class (second arg) element-type replace-both)))
	 ;; Default values for optional/key arguments are treated
	 ;; specially for array methods.
	 (if (and (listp arg) (numberp (second arg)))
	     (let ((actual-type
		    (grid:cffi-cl
		     (grid:st-type (find (first arg) carg-actual :key 'grid:st-symbol)))))
	       (list (first arg) ; Optional arg default numerical value
		     (if actual-type
			 ;; coerce to the right type
			 (coerce (second arg) actual-type)
			 (second arg))))
	     (if (listp arg)
		 (if (and (listp (second arg))
			  (eq (first (second arg)) 'eltcase))
		     ;; "eltcase" switch to the matching form
		     `(,(first arg)
			,(element-type-select (rest (second arg)) element-type))
		     ;; 'element-type is bound to the element-type.
		     (list
		      (first arg)
		      (subst `',element-type 'element-type
			     (second arg))))
		 arg)))))

(defun element-type-select (form element-type)
  "Find the actual form to use as the default based on the list in form."
  (loop for (type result) on form by #'cddr
     thereis (when (subtypep element-type type) result)))

(defun actual-array-c-type (category c-arguments &optional map-down)
  "Replace the declared proto-type with an actual GSL struct type
   in the GSL function name."
  (if map-down
      ;; If there are multiple C argument lists due to an optional
      ;; argument, then map this function onto each.
      (mapcar (lambda (carg) (actual-array-c-type category carg nil))
	      c-arguments)
      c-arguments))

(defun actual-element-c-type (element-type c-arguments)
  "Replace the generic element type :element-c-type with the
   actual element type."
  (mapcar 
   (lambda (v)
     (subst (grid:cl-cffi element-type) :element-c-type
	    (subst (grid:component-type element-type) :component-float-type v)))
   c-arguments))
