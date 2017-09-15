;; Physical attributes that describe (or partially describe) a physical state.
;; Liam Healy Mon Apr  1 2002 - 16:27
;; Time-stamp: <2013-05-26 19:01:14EDT state.lisp>

;; Copyright 2011, 2013 Liam M. Healy
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

(export '(components constraints
	  defattribute attributep format-attribute-name state-value show-state
	  make-physical-state make-new-physical-state
	  make-state-table set-state-value defined-attributes
	  copy-state map-state
	  *default-sfg* *return-formula* state-formula
	  add-attribute-to-parameter
	  make-parameter-with-attribute))

;;;;****************************************************************************
;;;; Attributes
;;;;****************************************************************************

;;; An attribute is a property of a physical system.  Information about it is stored in the same place as the information about physical dimensions.  When collected together, the attributes are stored in a state.  Unlike slots in an object, it is not assumed that all are defined; they will only be defined when they are needed.
;;; Example: rectangle: width, height, diagonal, area.

(defmacro tex-form (name)
  "How to print in TeX."
  `(unit-property ,name 'tex))

(defmacro description (name)
  "Description of the attribute."	; or unit, dimension,?
  `(unit-property ,name 'description))

#+not-implemented
(defmacro format-precision (name)
  "Bias on the significant-figures or fracpart-digits in printing.
   A list for two elements which are each a number or NIL."
  `(unit-property ,name 'precision))

(defmacro components (name)
  "Components of vector."
  `(unit-property ,name 'components))

(defmacro constraints (name)
  "Constraint on acceptable values."
  `(unit-property ,name 'constraints))

(defun apply-constraints (attribute value)
  "Apply the appropriate constraints, if any.
   This may constitute computing some mathemtical function of the
   number and using the result, or signalling an error."
  (case (constraints attribute)
    (:norm (normalize-angle value))	; note no check on angle dimension
    (:norm+ (normalize-angle value t))
    (t value)))

(defvar *all-attributes* nil "All defined attributes.")

(defmacro defattribute
    (name dimension
	  &optional synonyms print-names description components constraints
	  (make-function t))
  "Define an attribute of a physical system.  The name should
   not be the same as a physical dimension or unit name."
  `(progn
    (check-unames ',name ',synonyms)
    (pushnew ',name *all-attributes*)
    (setf (dimension ',name) ',dimension
     (synonyms ',name) ',synonyms
     (print-name ',name) ',print-names
     ;;(format-precision ',name) ',precision
     (constraints ',name) ',constraints
     (description ',name) ,description
     (components ',name) ',components
     (canonical-name ',name) ',name
     ,@(mapcan (lambda (s) `((canonical-name ',s) ',name)) synonyms))
    ,(if make-function
	 `(setf (fdefinition ',name) (lambda (state) (state-value ',name state))))))

(defun attributep (symbol-or-list &optional error)
  "This is a defined attribute, or list of attributes."
  (if (symbolp symbol-or-list)
      (or (find symbol-or-list *all-attributes*)
	  (when error
	    (error "~a is not defined as an attribute." symbol-or-list)))
    (mapcar (lambda (s) (attributep s error)) symbol-or-list)))

(defun format-attribute-name
    (name &optional (separate-components t) include-units (include-name t))
  "Format the attribute name, and possibly the units."
  (when name
    (flet ((add-units (attnm)
	     (format
	      nil
	      "~@[~a~]~@[ (~a)~]"
	      (when include-name attnm)
	      ;; The unit names, for when attribute values are printed as numbers
	      ;; alone (e.g., in a table).
	      (let ((expr
		      ;; ignore-errors so that non-attributes
		      ;; like 'timepoint will pass
		      (ignore-errors
		       (find-unit-expr (dimension name)))))
		(when (and include-units expr)
		  (with-output-to-string (stream)
		    (format-units nil expr t stream)))))))
      (if (and separate-components (components name))
	  ;; Format each component separately
	  ;; (set-nf-options style :tex) doesn't do subscripts yet.
	  (mapcar
	   (lambda (comp)
	     (add-units
	      (format nil "~a ~a"
		      (format-attribute-name name nil nil) comp)))
	   (components name))
	  ;; The attribute
	  (add-units
	   (if texstyle
	       ;; TeX print name of attribute
	       (tex-print-name name)
	       ;; plain print name
	       (if (or (eq (nf-option :style) :short)
		       (and (typep (nf-option :style) 'fixnum)
			    (cl:> (length (long-print-name name))
				  (nf-option :style))))
		   (short-print-name name)
		   (long-print-name name))))))))

;;;;****************************************************************************
;;;; State table
;;;;****************************************************************************

;;; A state is a collection of attributes about a particular physical system
;;; that is a partial or complete representation of a particular state.

;;; Rather than have attributes collected into formal state class instances like
;;; kepler-elements, it is better to keep an attribute bag that forms the argument 
;;; of functions.  This is because 
;;;  1) minor variations are very easy - e.g., use true anomaly instead of mean anomaly,
;;;  2) sometimes we don't have a complete state, just partial information,
;;;  3) we can memoize computed result back into the bag.
;;; Drawbacks are that the "generic function" has to be constructed on the fly,
;;; because it doesn't know what it's going to get as an argument.  

;;; Thus, we define a state-table, a table of attributes, e.g:
;;;   semimajor-axis   #_2.3_ER
;;;   eccentricity       0.345
;;;   epoch-time       #d2002-04-03T12:34:56
;;;   perigee-radius   #_1.5065_ER
;;; etc.

;;; Defgeneric?
(defun make-state-table ()
  (make-hash-table))

(defun set-state-value (value attribute state-table)
  "Set the value of a particular attribute for this state."
  (let ((attr (get-canonical-name attribute)))
    (if (dimension attr)
	(let ((st (or state-table (make-state-table))))
	  (setf (gethash attr st)
	    (apply-constraints
	     attr
	     (make-pq value (dimension attr)))))
      (error "Attribute ~a unknown." attribute))))

(defun (setf state-value) (value attribute state-table)
  (set-state-value value attribute state-table))

(defgeneric defined-attributes (state)
  (:documentation "List the attributes defined for this state.")
  (:method ((state hash-table))
	   (let ((attributes nil))
	     (maphash
	      (lambda (k v) (declare (ignore v)) (push k attributes))
	      state)
	     attributes))
  (:method ((state null)) nil))

(defgeneric show-state (state)
  (:documentation "Show the physical state attributes.")
  (:method ((state hash-table))
	   (maphash
	    (lambda (k v) (format t "~&~a ~25t= ~d" (string-downcase k) v) k v)
	    state)))

(defun copy-state (original-state &rest attributes)
  "Make a new state with the same attributes as an existing state.
   The attributes specified are computed if needed."
  (let ((new (make-state-table)))
    (dolist (attr attributes new)
      (when (get-canonical-name attr)	; it's really an attribute
	(set-state-value
	 (state-value attr original-state)
	 attr new)))))

(defun map-state (original-state attributes &optional function)
  "Make a new state by mapping a function of (value attribute-name)
   on existing attributes.  If function is NIL, use existing value.
   A function, if specified, should return the new value.  If attributes
   is NIL, use all the defined attributes of the original state."
  (let ((new (make-state-table)))
    (dolist (attr (or attributes (defined-attributes original-state)) new)
      (when (get-canonical-name attr)	; it's really an attribute
	(let ((oldval (state-value attr original-state nil)))
	  (when oldval
	    (if function
		(set-state-value (funcall function oldval attr) attr new)
	      (set-state-value oldval attr new))))))))

(defun make-physical-state (state attributes &rest attributes-values)
  "From the given attributes and values, copy the requested attributes
   from the existing state if they exist, then define new attribute values."
  (let ((new (map-state state attributes)))
    (loop for (att val) on attributes-values by #'cddr
	do (set-state-value val att new))
    new))

(defun make-new-physical-state (&rest attributes-values)
  "Make a new physical state with the given attribute values."
  (apply #'make-physical-state nil nil attributes-values))

(defvar *default-sfg* nil
  "The default state function generator.")

(defparameter *return-formula* nil
  "Show the formula used in computing the attribute.")

(defun state-formula
    (attribute-canonical state-table
     &optional apply-to-values (state-function-generator *default-sfg*))
  "Find the formula for the attribute given the known quantities in state."
  (apply state-function-generator attribute-canonical
	 (loop for att being the hash-keys
	    in state-table collect att)
	 (when apply-to-values
	   (loop for val being the hash-value
	      in state-table collect val))))

(defgeneric state-value
    (attribute state &optional state-function-generator)
  (:documentation "Get the value of the attribute from the state table.
   If it is not present, compute it if possible with state-function-generator.
   If state-function-generator is nil, do not compute.")
  (:method
      (attribute (state-table hash-table)
       &optional (state-function-generator *default-sfg*))
    (let ((attr (get-canonical-name attribute)))
      (or (gethash attr state-table nil)
	  (when state-function-generator
	    (let* ((formula
		    (state-formula
		     attr state-table t state-function-generator))
		   (numerical-answer
		    (set-state-value (eval formula) attr state-table)))
	      (if *return-formula*
		  (values numerical-answer formula)
		  numerical-answer))))))
  (:method (attribute (state null) &optional state-function-generator)
    (declare (ignore attribute state-function-generator))
    nil))

;;;;****************************************************************************
;;;; Parameters
;;;;****************************************************************************

;; Future

#|
(defun set-attribute (struct attribute &optional description)
  (setf (parameter-attribute struct) attribute
	(parameter-documentation struct)
	(format nil "~a~@[  For ~a~]" (description attribute) description)
	;; Eventually, get the type from the attribute
	(parameter-type struct) 'physical-quantity)
  struct)

(defun add-attribute-to-parameter (category name attribute &optional description)
  "Declare that the parameter, already created, has the stated attribute."
  (set-attribute (find-parameter category name) attribute description))

(defun make-parameter-with-attribute
    (category name attribute &optional type description default)
  ;; It seems like the type should be defined in the attribute.
  "Make a parameter that has an attribute."
  (make-parameter category name :value default :type type)
  (set-attribute
   (find-parameter category name)
   attribute
   description))
|#

;;;;****************************************************************************
;;;; Broken
;;;;****************************************************************************

#|
;;; This is broken
(defmethod nf
    ((object hash-table) &optional (stream *standard-output*))
  (when (nf-readably)
    (format stream
	    "~<(let ((tbl ~<(make-hash-table ~
		~_~2i:test '~a ~
		~_~2i:size ~d ~
		~_~2i:rehash-size ~d ~
		~_~2i:rehash-threshold ~d)~>)) ~
        ~_~:i(setf ~:{~&(gethash '~(~a~) tbl) ~a~})
        tbl)~>"
	    (hash-table-test object)
	    (hash-table-size object)
	    (hash-table-rehash-size object)
	    (hash-table-rehash-threshold object)
	    (loop for k being the hash-key of object
		using (hash-value v)
		collect (list k (nf-string v))))))
|#


