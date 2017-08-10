;; Object (re)creation.
;; Liam Healy Sat Feb 23 2002 - 17:35
;; Time-stamp: <2014-11-05 12:06:11EST object.lisp>

;; Copyright 2011, 2014 Liam M. Healy
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
(export '(readably creation-form-readably creation-form creation-form-stream
	  print-readably *use-readable-print-form* def-make-load-form
	  serialize-binding))

(defun readably ()
  (or *print-readably* *print-escape*))

;; See notes [[id:2c2599e1-16b1-441a-a34f-6d831b9bfb95][Creation form readably, creation-form-using-reader-macros]]
;; Essentially: it doesn't make any sense to return the object, because that can't be put into a serializable object.
;; Leave in for now, just don't use this.
(defmacro creation-form-readably (object non-readably)
  "If we are printing readably, then rely on printed form to create the new object."
  `(if (readably)
       ,object
       ,non-readably))

;;; creation-form generates a form that creates the object,
;;; much like make-load-form, but can be used on objects of built-in class. 
(defgeneric creation-form (object)
  (:documentation "A form that creates the object.")
  (:method ((object t)) (make-load-form object))
  (:method ((object symbol)) `',object)
  (:method ((object number)) object)
  (:method ((object string)) object)
  (:method ((object vector))
    (creation-form-readably
     object
     `(vector ,@(map 'list #'creation-form object))))
  (:method ((object list))
    `(list ,@(mapcar #'creation-form object)))
  (:method ((object hash-table))
    `(let ((tbl (make-hash-table
		 :test ',(hash-table-test object)
		 :size ,(hash-table-size object)
		 :rehash-size ,(hash-table-rehash-size object)
		 :rehash-threshold ,(hash-table-rehash-threshold object))))
       (setf ,@(let ((pairs (list 0)))
		 (maphash
		  (lambda (k v)
		    (nconc pairs (list `(gethash ',k tbl)
				       (creation-form v))))
		  object)
		 (rest pairs)))
       tbl)))

(defmacro def-make-load-form (object-type)
  "Defmethod the make-load-form assuming creation-form exists."
  `(defmethod make-load-form ((object ,object-type) &optional env)
     (declare (ignore env))
     (let ((*print-escape* nil) (*print-readably* nil))
       (creation-form object))))

(defun print-readably (object &optional (stream t))
  "If the CL-defined specials are defined appropriately, print readably
   and return T, otherwise, return NIL."
  (when (and *read-eval* (readably))
    (format stream "#.~s" (creation-form object))
    t))

(defparameter *use-readable-print-form* t
  "Use the readable print form in creation-form if available.")

(defmacro serialize-binding (symbol)
  "Generate a form that when loaded will redefine the object
   and bind the variable to it."
  `'(defparameter ,symbol ,(creation-form (symbol-value symbol))))
