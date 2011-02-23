;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;


;;; Designators describe entities such as objects, locations or
;;; actions by a set of properties. When the decision is made that two
;;; designators describe the same entity, they must be equated. They
;;; get linked to each other and therefore represent the course of the
;;; entity over time.

(in-package :desig)

(define-condition designator-error (simple-error)
  ())

(defclass designator ()
  ((timestamp :reader timestamp :initform (current-timestamp)
              :documentation "Timestamp of creation of reference or nil
                             if still not referencing an object.")
   (description :reader description :initarg :description
                :documentation "List of properties describing the designator.")
   (parent :reader parent :initarg :parent :initform nil
           :documentation "The parent designator, i.e. the designator
                          used to create this designator, or nil.")
   (successor :reader successor :initform nil
              :documentation "The successor designator this designator
                              is parent of.")
   (valid :reader valid :initform nil
          :documentation "Returns true if the designator is valid,
                         i.e. its reference has already been computed.")
   (data :initform nil
         :documentation "Data this designator describes or nil if the
                        designator was resolved yet.")))

(defgeneric make-designator (class description &optional parent)
  (:documentation "Returns a new designator of type `class', matching
                   `description'. If `parent' is specified, the new
                   designator is equated to parent."))

(defgeneric equate (parent successor)
  (:documentation "Equates `successor' with `parent', i.e. makes them
                   describe the same entity. Returns successor."))

(defgeneric desig-equal (desig-1 desig-2)
  (:documentation "Returns T if desig-1 and desig-2 describe the same
                   entity, i.e. if they have been equated before."))

(defgeneric first-desig (desig)
  (:documentation "Returns the first ancestor of `desig'."))

(defgeneric current-desig (desig)
  (:documentation "Returns the current, i.e. the youngest designator
                   that has been equated to `desig'."))

(defgeneric reference (desig)
  (:documentation "Computes and/or returns the lisp object this
                   designator references. Note: this method _MUST_ be
                   deterministic, i.e. it must always return the same
                   object."))

(defgeneric next-solution (desig)
  (:documentation "Returns a new designator that points to a different
                   object but matches the same description or nil if
                   no other solutions can be found.. This method is
                   ment for dealing with ambiguities in designator
                   descriptions."))

(defvar *designator-pprint-description* t
  "If set to T, DESIGNATOR objects will be pretty printed with their description.")

(defmethod print-object ((object designator) stream)
  (print-unreadable-object (object stream :type t :identity t)
    (when *designator-pprint-description*
      (write (description object) :stream stream))))

(defmethod equate ((parent designator) (successor designator))
  (assert (not (desig-equal parent successor)) ()
          "Cannot equate designators that are already equal.")
  (let ((youngest-parent (current-desig parent))
        (first-parent (first-desig parent)))
    (when (parent successor)
      (setf (slot-value first-parent 'parent) (parent successor))
      (setf (slot-value (parent first-parent) 'successor) first-parent))
    (setf (slot-value successor 'parent) youngest-parent)
    (setf (slot-value youngest-parent 'successor)
          successor))
  successor)

(defmethod desig-equal ((desig-1 designator) (desig-2 designator))
  (eq (first-desig desig-1)
      (first-desig desig-2)))

(defmethod reference :after ((desig designator))
  (setf (slot-value desig 'valid) t))

(defmacro register-designator-class (type class-name)
  "Registers a class as a designator class so that it can be used
together with MAKE-DESIGNATOR and WITH-DESIGNATORS"
  `(pushnew (cons ',type ',class-name) (get 'make-designator :desig-types) :key #'car))

(defmethod make-designator ((type symbol) description &optional parent)
  (let ((desig-class-name (cdr (assoc type (get 'make-designator :desig-types)))))
    (assert desig-class-name () (format nil "Designator type `~a' undefined" type))
    (make-designator (find-class desig-class-name)
                     description parent)))

(defmethod make-designator ((type standard-class) description &optional parent)
  (let ((desig (make-instance type
                 :description description)))
    (when parent
      (equate parent desig))
    desig))

(defmethod first-desig ((desig designator))
  (if (null (parent desig))
      desig
      (first-desig (parent desig))))

(defmethod current-desig ((desig designator))
  "gets the last know successor in chain, being the most current"
  (if (null (successor desig))
      desig
      (current-desig (successor desig))))

;;; TODO: Make with-designators use language features. We need a
;;; transparent with-designator macro.
(defmacro with-designators (defs &body body)
  `(let* ,(mapcar (lambda (def)
                    (destructuring-bind (name (type props)) def
                      `(,name (make-designator ',type ,props))))
                  defs)
     ,@body))
