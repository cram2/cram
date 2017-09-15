;; Methods for grid functions
;; Liam Healy 2009-12-21 11:19:00EST methods.lisp
;; Time-stamp: <2013-12-28 12:12:51EST methods.lisp>
;;
;; Copyright 2009, 2010, 2012 Liam M. Healy
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

(in-package :grid)

(defmethod rank ((object foreign-array))
  (length (dimensions object)))

;;;;****************************************************************************
;;;; Reference to elements: macros
;;;;****************************************************************************

(defun linearized-index (object indices)
  (affi:calculate-index (affi object) indices))

;;; Define an "internal" function without any declaration reading and have
;;; macros call it.  These macros could read declarations.
;;; aref/aref*/setf could use those macros too.

(eval-when (:compile-toplevel :load-toplevel :execute)

(defun farray-element-type (type-declaration)
  (lookup-type
   (if (listp type-declaration) (first type-declaration) type-declaration)
   *class-element-type*
   nil "Foreign array"))

(defun faref-int (object type-declaration indices)
  "Access the foreign array element.  The more details provided in
   type-declaration, the faster access will be."
  ;; No (the... ) declaration on scalar output because that would mess up setf form?
  `(cffi:mem-aref
    (the ,+foreign-pointer-type+ (slot-value ,object 'foreign-pointer))
    ,(if type-declaration
	 `',(cl-cffi (farray-element-type type-declaration))
	 `(cl-cffi (element-type ,object)))
    (the fixnum
      ,(if (rest indices)
	   ;; Rank>1
	   (if (and (listp type-declaration) (rest type-declaration))
	       `(cl:+ ,@(mapcar
			 (lambda (co in)
			   (if (eql co 1) in `(cl:* ,co ,in)))
			 (affi::row-major-coeff-from-dimensions (rest type-declaration))
			 indices))
	       (if (rest indices)
		   `(linearized-index ,object ,(cons 'list indices))
		   (first indices)))
	   ;; Rank=1, i.e. vector, no dimensions needed
	   (first indices)))))

(defun wrap-the-decl (form type-decl &optional (do-it t))
  (if do-it
      `(the ,type-decl ,form)
      form))

(defun wrap-set-thedecl
    (object element-type wrap-the-decl value set-value)
  "Return the object if it's not a complex; if it is a complex, assume
   it's a pointer and convert to a CL complex."
  (wrap-the-decl
   (if set-value
       `(setf ,object ,value)
       object)
   element-type
   (and element-type wrap-the-decl)))
)

(defmacro faref (object type-declaration &rest indices)
  "Macro to get an element of the foreign array; type-declaration can
  be nil, a foreign array type, or a list of foreign-array type and
  dimension(s).  The more details provided at macroexpansion time,
  the faster the result at run time."
  (wrap-set-thedecl
   (faref-int object type-declaration indices)
   (when type-declaration (farray-element-type type-declaration))
   t nil nil))

(defmacro set-faref (value object type-declaration &rest indices)
  "Macro to set an element of the foreign array; type-declaration can
  be nil, a foreign array type, or a list of foreign-array type and
  dimension(s).  The more details provided at macroexpansion time,
  the faster the result at run time."
  (wrap-set-thedecl
   (faref-int object type-declaration indices)
   (when type-declaration (farray-element-type type-declaration))
   t value t))

#|
;;; Macroexpansions:

(macroexpand '(faref foo nil 3))
(IF (SUBTYPEP (ELEMENT-TYPE FOO) 'COMPLEX)
    (FOREIGN-STRUCTURES-BY-VALUE:OBJECT
     (CFFI:MEM-AREF
      (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
      (CL-CFFI (ELEMENT-TYPE FOO)) (THE FIXNUM 3))
     (CL-CFFI (ELEMENT-TYPE FOO)))
    (CFFI:MEM-AREF
     (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
     (CL-CFFI (ELEMENT-TYPE FOO)) (THE FIXNUM 3)))

(macroexpand '(faref foo vector-double-float 3))
(THE DOUBLE-FLOAT
     (CFFI:MEM-AREF
      (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
      ':DOUBLE (THE FIXNUM 3)))

(macroexpand '(faref foo vector-complex-double-float 3))
(THE (COMPLEX DOUBLE-FLOAT)
     (CFFI:MEM-AREF
      (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
      '(:STRUCT COMPLEX-DOUBLE-C) (THE FIXNUM 3)))

(macroexpand '(faref foo matrix-double-float i j))
(THE DOUBLE-FLOAT
     (CFFI:MEM-AREF
      (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
      ':DOUBLE (THE FIXNUM (LINEARIZED-INDEX FOO (LIST I J)))))

(macroexpand '(faref foo (matrix-double-float 4 5) i j))
(THE DOUBLE-FLOAT
     (CFFI:MEM-AREF
      (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
      ':DOUBLE (THE FIXNUM (+ (* 5 I) J))))

(macroexpand '(faref foo (matrix-complex-double-float 4 5) i j))
(THE (COMPLEX DOUBLE-FLOAT)
     (CFFI:MEM-AREF
      (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
      '(:STRUCT COMPLEX-DOUBLE-C) (THE FIXNUM (+ (* 5 I) J))))

(macroexpand-1 '(set-faref val foo nil i j))
(SETF (CFFI:MEM-AREF
       (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
       (CL-CFFI (ELEMENT-TYPE FOO))
       (THE FIXNUM (LINEARIZED-INDEX FOO (LIST I J))))
        VAL)

(macroexpand-1 '(set-faref val foo matrix-complex-double-float i j))
(THE (COMPLEX DOUBLE-FLOAT)
     (SETF (CFFI:MEM-AREF
            (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
            '(:STRUCT COMPLEX-DOUBLE-C)
            (THE FIXNUM (LINEARIZED-INDEX FOO (LIST I J))))
             VAL))

(macroexpand-1 '(set-faref val foo (matrix-double-float 4 5) i j))
(THE DOUBLE-FLOAT
     (SETF (CFFI:MEM-AREF
            (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
            ':DOUBLE (THE FIXNUM (+ (* 5 I) J)))
             VAL))

(macroexpand-1 '(set-faref val foo (matrix-complex-double-float 4 5) i j))
(THE (COMPLEX DOUBLE-FLOAT)
     (SETF (CFFI:MEM-AREF
            (THE SB-SYS:SYSTEM-AREA-POINTER (SLOT-VALUE FOO 'FOREIGN-POINTER))
            '(:STRUCT COMPLEX-DOUBLE-C) (THE FIXNUM (+ (* 5 I) J)))
             VAL))

|#

;;;;****************************************************************************
;;;; Reference to elements: methods
;;;;****************************************************************************

(defmethod aref* ((object foreign-array) linearized-index)
  (faref object nil linearized-index))

(defmethod aref ((object foreign-array) &rest indices)
  (faref object nil (linearized-index object indices)))

(defmethod (setf aref*) (value (object foreign-array) linearized-index)
  (set-faref value object nil linearized-index))

(defmethod (setf aref) (value (object foreign-array) &rest indices)
  (set-faref value object nil (linearized-index object indices)))

;;; The following group of specific methods override the above methods
;;; because with much faster forms for specific element types, a fact
;;; reported by Sebastian Sturm.  See
;;; http://common-lisp.net/pipermail/cffi-devel/2007-September/002696.html.

#.`(progn
     ,@(loop for rank from 1 to 2
	  append
	  (loop for element-type in *array-element-types*
	     for array-type = (data-class-name rank element-type)
	     append
	     `((defmethod aref ((object ,array-type) &rest indices)
		 ,(case rank
			(1 `(faref object ,array-type (first indices)))
			(2 `(faref object ,array-type (first indices) (second indices)))))
	       (defmethod aref* ((object ,array-type) linearized-index)
		 (declare (fixnum linearized-index))
		 (faref object ,array-type linearized-index))
	       (defmethod (setf aref) (value (object ,array-type) &rest indices)
		 (declare (type ,element-type value))
		 ,(case rank
			(1 `(set-faref value object ,array-type (first indices)))
			(2 `(set-faref value object ,array-type (first indices) (second indices)))))
	       (defmethod (setf aref*) (value (object ,array-type) linearized-index)
		 (declare (fixnum linearized-index) (type ,element-type value))
		 (set-faref value object ,array-type linearized-index))))))

(defun canonical-foreign-element-type (element-type)
  "Return the given element type, possibly translating to a canonical name."
  (case element-type
    (fixnum (list 'signed-byte #+int64 64 #+int32 32))
    (t element-type)))

(defmethod make-grid-data
    ((type (eql 'foreign-array)) dimensions rest-spec
     &rest keys &key &allow-other-keys)
  (let* ((element-type
	   (canonical-foreign-element-type (spec-element-type rest-spec)))
	 (array
	  (apply
	   'make-instance
	   ;; Make as the specific subclass of foreign-array,
	   ;; e.g. vector-double-float.
	   (data-class-name (length dimensions) element-type)
	   :dimensions dimensions
	   :element-type element-type
	   keys)))
    array))

(defmethod copy ((object foreign-array)
		 &rest args
		 &key specification grid-type dimensions element-type
		 destination)
  (declare (ignorable specification grid-type dimensions element-type
		      destination))
  (apply 'copy-grid object args))

;;;;****************************************************************************
;;;; Compiler macros
;;;;****************************************************************************

;;;; These compiler macros are designed to speed things up when the type
;;;; has been declared, so that a direct expansion into cffi:mem-aref
;;;; can be made.

;;;; The declaration forms (declare (type ....)) are for SBCL and CCL
;;;; only, though in principle they can be extended to anything that
;;;; supports the CLtL2 function #'variable-information.

;;; Need to shadow cl:setf and expand it without rebinding the object
;;; to see declarations.

(eval-when (:compile-toplevel :load-toplevel :execute)
  #+sbcl (require :sb-cltl2))

(eval-when (:compile-toplevel :load-toplevel :execute)
(defparameter *accelerated-aref-types* 
    (all-types *class-element-type*)
    "A list of grid types whose access through aref or aref* will be
  directly translated into cffi:mem-aref calls for speed if the
  compiler supports it.")

(defun declared-type-dimensions (variable env)
  "The declaration of type and dimensions for a foreign array, or NIL otherwise."
  (when (symbolp variable)
    (let ((type-decl
	   (rest (assoc 'type
			(nth-value
			 2
			 #+(or sbcl ccl) ; any implementation that supports variable-information
			 (#+sbcl sb-cltl2:variable-information
				 #+ccl ccl:variable-information
				 variable env)
			 #-(or sbcl ccl)
			 nil)))))
      (when (member
	     (if (listp type-decl) (first type-decl) type-decl)
	     *accelerated-aref-types*)
	type-decl))))

(defun extract-the-type-specifier (form)
  "When something is declared with a 'the special operator,
   return the item and type specifier."
  (if (and (listp form) (eq 'the (first form)))
      (values (third form) (second form))
      (values form nil)))

(defun using-declared-type (whole-form when-declared-expander object arg env)
  "Look for either 'declare or 'the declarations on the object, and expand accordingly if declared, or else return the whole-form.  The second value returned indicates whether there was a replacement; in the former case it is T, in the latter NIL."
  (or (multiple-value-bind (form type)	; used a 'the form
	  (extract-the-type-specifier object)
	(when type
	  (alexandria:once-only (form)
	    (values (funcall when-declared-expander form type arg) t))))
      (let ((decl (declared-type-dimensions object env)))
	(when decl (values (funcall when-declared-expander object decl arg) t)))
      (values whole-form nil)))

(defun select-aref-expander (form type args)
  (cond
    ((subtypep type 'grid:foreign-array) `(faref ,form ,type ,@args))
    ((subtypep type 'cl:array)  `(cl:aref ,form ,@args))))

(defun expand-faref (form type args)
  `(faref ,form ,type ,@args))

(defun expand-set-faref (form type args)
  `(set-faref ,(first args) ,form ,type ,@(rest args)))
)

(define-compiler-macro aref* (&whole form object linearized-index &environment env)
  "Expand the aref* form directly into a faster call if the type is known at compile time."
  (using-declared-type form 'expand-faref object (list linearized-index) env))

(define-compiler-macro aref (&whole form object &rest indices &environment env)
  "Expand the aref form directly into a faster call if the type is known at compile time."
  (using-declared-type form 'expand-faref object indices env))

(define-compiler-macro (setf aref*)
    (&whole form value object linearized-index &environment env)
  (using-declared-type form 'expand-set-faref object (list value linearized-index) env))

(define-compiler-macro (setf aref)
    (&whole form value object &rest indices &environment env)
  (using-declared-type form 'expand-set-faref object (cons value indices) env))

#|
;;; Tests

(funcall (compiler-macro-function 'aref*) '(aref* (the vector-double-float zzz) 3) nil)
(LET ((#:FORM1278 ZZZ))
  (FAREF #:FORM1278 VECTOR-DOUBLE-FLOAT 3))

(funcall (compiler-macro-function 'aref*) '(aref* zzz 3) nil)
(AREF* ZZZ 3)

(funcall (compiler-macro-function '(setf aref*))
	 '(setf (aref* (the vector-double-float zzz) 3) value) nil)
error while parsing arguments to DEFINE-COMPILER-MACRO (SETF AREF*):
  invalid number of elements in
    ((AREF* # 3) VALUE)
  to satisfy lambda list
    (&WHOLE FORM VALUE OBJECT LINEARIZED-INDEX &ENVIRONMENT
     ENV):
  exactly 3 expected, but 2 found
   [Condition of type SB-KERNEL::ARG-COUNT-ERROR]

(funcall (compiler-macro-function '(setf aref*))
	 '(funcall #'(setf aref*) value (the vector-double-float zzz) 3) nil)
(LET ((#:FORM1282 ZZZ))
  (SET-FAREF VALUE #:FORM1282 VECTOR-DOUBLE-FLOAT 3))

(funcall (compiler-macro-function 'aref*) '(aref* boo 3) nil)

(declaim (type (matrix-double-float 12 12) goo))
(funcall (compiler-macro-function 'aref) '(aref goo i j) nil)
(FAREF GOO (MATRIX-DOUBLE-FLOAT 12 12) I J)

|#

;;;;****************************************************************************
;;;; The setf workaround
;;;;****************************************************************************

;;; In order to benefit from setf macros, you must funcall (setf aref)
;;; or (setf aref*)
;;; rather than use the setf macro, because the compiler may rebind
;;; the arguments to new (undeclared) variables in the expansion, and
;;; this compiler macro won't know about them.  A workaround would be
;;; to shadow cl:setf and predetect the aref*, then expand without
;;; rebinding the grid variable.
;;; [[id:24f28096-0581-489b-96bb-43f8e397d0d8][Compiler macros and setf]]

;;; Also, I'm not sure if my (setf aref) and (setf aref*) compiler
;;; macros are being expanded anyway.

(eval-when (:compile-toplevel :load-toplevel :execute)
(defun my-setf-pair (thing value)
  (if (and (listp thing) (member (first thing) '(aref aref*)))
      `(funcall #'(setf ,(first thing)) ,@(cons value (rest thing)))
      `(cl:setf ,thing ,value)))
)

;;; In lieu of shadowing 'setf, we define gsetf
(export 'gsetf)
(defmacro gsetf (&rest args)
  (cons 'progn
	(loop for (thing value) on args by #'cddr
	   collect (my-setf-pair thing value))))

;;;;****************************************************************************
;;;; Simple math functions
;;;;****************************************************************************

(defmethod antik:zerop ((grid foreign-array))
  (loop for ind from 0 below (total-size grid)
	always (antik:zerop (aref* grid ind))))
