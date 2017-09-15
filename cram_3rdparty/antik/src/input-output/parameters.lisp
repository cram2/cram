;; Global/local parameters to pass to functions
;; Liam Healy 2013-02-16 20:51:14HST variable-metadata.lisp
;; Time-stamp: <2014-01-08 22:28:46EST parameters.lisp>

;; Copyright 2011, 2013, 2014 Liam M. Healy
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

(export '(make-parameter define-parameter parameter-value
	  set-parameter
	  set-parameters with-parameters parameter-help
	  make-parameters-from-table))

;;;;****************************************************************************
;;;; Options
;;;;****************************************************************************

(defparameter *earmuffs* nil "Create special variable symbols with asterisks at the beginning and end of names. If t, a synonym will be created that is the same symbol without the earmuffs.")

;;;;****************************************************************************
;;;; Storage and utility
;;;;****************************************************************************

(defvar *categories* nil "Categories of parameters, which correspond to packages.")

(defvar *parameters* nil "A list of the parameters (canonical names only) as symbols.")

(defvar *symbols* (make-hash-table)
  "Table of synonyms and corresponding canonical parameter names.")

(defun ensure-package (package)
  "If the non-nil package designator designates an existing package, return it. If it designates a non-existent package, make and return it. Otherwise, return the current package."
  ;; Check that we're not trying to use the keyword, etc. package?
  (or (when package (or (find-package package) (make-package package)))
      *package*))

(defmacro canonical-parameter-name (symbol)
  `(gethash ,symbol *symbols*))

(defun add-symbol-to-table (symbol synonym &optional (redefine-action :warning))
  "Add the synonym to the hash table."
  ;; Subtle problem: if there is a synonym for some parameter and then the canonical name is used as a synonym for a new parameter, using the original synonym will cause a malfunction.
  (when (and
	 redefine-action
	 (canonical-parameter-name synonym)
	 (not (eql (canonical-parameter-name synonym) symbol)))
    (case redefine-action
      (:warning (warn "Symbol ~a is already defined as ~a; redefining"
		      synonym
		      (canonical-parameter-name synonym)))
      (:error (cerror
	       "Redefine the symbol"
	       "Symbol ~a is already defined as ~a"
	       symbol
	       (canonical-parameter-name symbol)))))
  (setf (canonical-parameter-name synonym) symbol))

(defun add-symbols-to-table (symbol synonyms)
  "Add the synonyms to the hash table."
  (mapcar (alexandria:curry 'add-symbol-to-table symbol) synonyms))

(defun find-parameter-symbol (category name)
  (let* ((*package* (ensure-package category))
	 (symbol (alexandria:symbolicate name)))
    (values
     (if *earmuffs* (alexandria:symbolicate #\* name #\*) symbol)
     symbol)))

(defun find-category (name)
  (if (member (find-package name) *categories*) (find-package name)))

(defun category-list ()
  (mapcar 'package-name *categories*))

(defun parameter-list (category)
  (let ((pkg (find-category category)))
    (when pkg
      (remove-if-not (lambda (p) (eq (symbol-package p) pkg)) *parameters*))))

(defun find-parameter (category name)
  "Find the registered parameter."
  (let ((symbol (find-parameter-symbol category name)))
    (multiple-value-bind (can found)
	(canonical-parameter-name symbol)
      (if found can
	  (error "Parameter ~a:~a not defined." category name)))))

;;;;****************************************************************************
;;;; Definition and use
;;;;****************************************************************************

(defun parameter-symbol-create (category name synonyms)
  "Create and export the symbol used to represent the parameter, including possibly making the appropriate package (category)."
  (multiple-value-bind (symbol sym-plain)
      (find-parameter-symbol category name)
    (let* ((syns (mapcar (alexandria:curry 'find-parameter-symbol category) synonyms)))
      (pushnew symbol syns)
      (pushnew sym-plain syns)
      (ensure-package category)
      (pushnew (find-package category) *categories*)
      (export syns (package-name (symbol-package symbol)))
      (values symbol syns))))

(defun parameter-post-definition
    (symbol syns documentation type type-supplied-p value value-supplied-p attribute attribute-supplied-p)
  (setf (documentation symbol 'variable) documentation)
  (when type-supplied-p (setf (get symbol :type) type))
  (when value-supplied-p
    (check-parameter-type symbol value t)
    (set symbol value))
  (when attribute-supplied-p (setf (get symbol :attribute) attribute))
  (pushnew symbol *parameters*)
  (add-symbols-to-table symbol (cons symbol syns)))

(defun make-parameter
    (category name
     &key (value nil value-supplied-p) (type nil type-supplied-p) (attribute nil attribute-supplied-p) documentation synonyms)
  "Define the parameter (global special variable) with the given name.  It will be defined in the package named by category (which will be created if necessary), and exported from it."
  (multiple-value-bind (symbol syns)
      (parameter-symbol-create category name synonyms)
    (proclaim `(special ,symbol))
    (parameter-post-definition
     symbol syns documentation type type-supplied-p value value-supplied-p attribute attribute-supplied-p)))

(defmacro define-parameter
    (category name
     &key (value nil value-supplied-p) (type nil type-supplied-p) (attribute nil attribute-supplied-p) documentation synonyms)
  "Define the parameter (global special variable) with the given name.  It will be defined in the package named by category (which will be created if necessary), and exported from it."
  (multiple-value-bind (symbol syns)
      (parameter-symbol-create category name synonyms)
    `(eval-when (:compile-toplevel :load-toplevel :execute)
       (parameter-symbol-create ',category ,name ,synonyms)
       ;;,export-form
       (defvar ,symbol)
       (parameter-post-definition
	',symbol ',syns ,documentation ',type ,type-supplied-p ,value ,value-supplied-p ,attribute ,attribute-supplied-p))))

;;; Test definition
#+(or)
(define-parameter :nf :style :value 20
  :type (or null fixnum (member :tex :short :readable))
  :synonyms (:foo)
  :documentation "Style of format, plain (nil), LaTeX (:tex), or plain but")

;;;;****************************************************************************
;;;; Bind parameters to values
;;;;****************************************************************************

(defmacro parameter-value (category name)
  "Get or set the parameter value."
  `(symbol-value ',(find-parameter category name)))

(defparameter *parameter-check-function* 'typep)

(defun check-parameter-type (symbol value error)
  (unless (funcall *parameter-check-function* value (get symbol :type))
    (when error 
      (error "Parameter ~a value ~a is not the required type ~a"
	     symbol value (get symbol :type)))))

(defmacro set-parameter-value* (symbol value)
  "Set the parameter value, checking that the value is of the correct type."
  (alexandria:once-only (value)
    `(progn (check-parameter-type ',symbol ,value t)
	    (setf ,symbol ,value))))

(defmacro set-parameter (category name value)
  "Set the parameter value, checking that the name is legitimate and the value is of the correct type."
  `(set-parameter-value* ,(find-parameter category name) ,value))

(defsetf parameter-value set-parameter)

;; (parameter-value :nf :foo)
;; (setf (parameter-value :nf :foo) 33)

(defmacro set-parameters (category &rest names-values)
  "Set the values of the parameters."
  `(setf 
    ,@(loop for (name value) on names-values by #'cddr
	 append `((parameter-value ,category ,name) ,value))))

;;; eventually use metabang-bind
;;; Change from previous: this uses let-style names-values.
(defmacro with-parameters ((category &rest name-values) &body body)
  "Provide local scope for parameter values, and possibly bind new values."
  (multiple-value-bind (setvals rhs-values)
      (loop for nv in name-values
	    for symb = (gensym "RHS")
	    when (listp nv)
	      collect (list symb (second nv)) into rhs-values
	    when (listp nv)
	      append (list `(parameter-value ,category ,(first nv)) symb) into setvals
	    finally (return (values setvals rhs-values)))
    `(let ,rhs-values
       (let (,@(mapcar
		 (lambda (nm)
		   (if (listp nm)
		       (find-parameter category (first nm))
		       (list (find-parameter category nm) `(parameter-value ,category ,nm))))
		 name-values))
	 ,(when (some 'listp name-values)
	    `(setf ,@setvals))
	 ,@body))))

;;; 
#+(or)
(with-parameters (:nf (:style 44))
  (foobar (parameter-value :nf :style)))

#+(or)
(with-parameters (:nf (:zibble 44))
  (foobar (parameter-value :nf :zibble)))

;;;;****************************************************************************
;;;; Help
;;;;****************************************************************************

(defun format-list (list stream)
  (apply #'format stream
	 ;;; Shamelessly cribbed from the Hyperspec.
	 "~#[none~;~a~;~a and ~a~:;~@{~#[~; and ~]~a~^, ~}~]."
	 (sort list #'string-lessp)))

(defun parameter-help (&optional category name (stream t))
  "Print all information known about the parameter.  If category is nil (default), names of all categories are printed.  If name is nil, all defined parameters in that category are printed."
  ;; If T passed for name, only parameters set to non-default values
  ;; will be printed?
  (if category
      (if name
	  (let ((par (find-parameter category name)))
	    (when par
	      (format stream "~&~a: ~a~&Type is ~s,~&Current value is ~s."
		      par
		      (documentation par 'variable)
		      (get par :type)
		      (symbol-value par))))
	  (format stream "~&Parameters in ~a: ~a"
		  category
		  (format-list (mapcar 'symbol-name (parameter-list category)) nil)))
      (format stream "~&Parameter categories: ~a"
	      (format-list (category-list) nil))))
