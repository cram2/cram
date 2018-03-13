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

(defvar *default-role* nil
  "Defines the default role to be used to resolve designators.")

(defvar *designators* (tg:make-weak-hash-table :weakness :key)
  "Weak hash table that contains all currently valid designators as
  keys. The values are unused.")

(define-condition designator-error (simple-error)
  ((designator :reader designator :initarg :designator :initform nil))
  (:default-initargs :format-control "DESIGNATOR-ERROR"))

(defclass designator ()
  ((timestamp :reader timestamp :initform nil
              :documentation "Timestamp of creation of reference or nil
                             if still not referencing an object.")
   (description :reader description :initarg :description
                :reader properties
                :documentation "List of properties describing the designator.")
   (parent :reader parent :initarg :parent :initform nil
           :documentation "The parent designator, i.e. the designator
                          used to create this designator, or nil.")
   (successor :reader successor :initform nil
              :documentation "The successor designator this designator
                              is parent of.")
   (effective :reader effective :initform nil
          :documentation "Returns true if the designator is an
          effective designator, i.e. it is grounded in either
          reasoning or sensor data and has a valid solution.")
   (data :initform nil
         :documentation "Data this designator describes or nil if the
                        designator was resolved yet.")
   (lock :initform (sb-thread:make-mutex)
         :documentation "Lock for synchronizing reference calls.")
   (quantifier :reader quantifier :initform :a
               :documentation "Quantifier can be :a, :an, :all etc.")))

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

(defgeneric reference (desig &optional role)
  (:documentation "Computes and/or returns the lisp object this
                   designator references. Note: this method _MUST_ be
                   deterministic, i.e. it must always return the same
                   object."))

(defgeneric next-solution (desig)
  (:documentation "Returns a new designator that points to a different
                   object but matches the same description or nil if
                   no other solutions can be found. This method is
                   meant for dealing with ambiguities in designator
                   descriptions."))

(defgeneric resolve-designator (desig role)
  (:documentation "Resolves the designator and generates an instance
  that is to be bound to the designator's reference slot. This is a
  lower-level method and should not be called directly. The method is
  used in REFERENCE. Through the `role' parameter, it provides an
  interface that allows to select the actual mechanism to resolve the
  designator. A default implementation is provided for the
  role 'DEFAULT-ROLE."))

(defgeneric designator-solutions-equal (solution-1 solution-2)
  (:documentation "Compares two designator solutions and returns T if
  they are equal.")
  (:method ((solution-1 t) (solution-2 t))
    (eql solution-1 solution-2))
  (:method ((solution-1 string) (solution-2 string))
    (string-equal solution-1 solution-2))
  (:method ((solution-1 list) (solution-2 list))
    (equal solution-1 solution-2))
  (:method ((solution-1 array) (solution-2 array))
    (equalp solution-1 solution-2)))

(defvar *designator-pprint-description* t
  "If set to T, DESIGNATOR objects will be pretty printed with their description.")

;; (defmethod print-object ((object designator) stream)
;;   (print-unreadable-object (object stream :type t :identity t)
;;     (when *designator-pprint-description*
;;       (write (description object) :stream stream))))

(defmethod print-object ((object designator) stream)
  (flet ((no-colon (keyword)
           (if (symbolp keyword)
               (intern (symbol-name keyword))
               keyword)))
   (print-unreadable-object (object stream :type nil :identity nil)
     (when *designator-pprint-description*
       (write (no-colon (quantifier object)) :stream stream)
       (write #\  :escape nil :stream stream)
       (write (no-colon (get-desig-class object)) :stream stream)
       (dolist (key-value (description object))
         (write #\linefeed :escape nil :stream stream)
         (write "    " :escape nil :stream stream)
         (write #\( :escape nil :stream stream)
         (write (no-colon (car key-value)) :stream stream)
         (write #\  :escape nil :stream stream)
         (write (no-colon (cadr key-value)) :stream stream)
         (write #\) :escape nil :stream stream))))))

(define-hook cram-utilities::on-equate-designators (successor parent))

(defmethod equate ((parent designator) (successor designator))
  (assert (not (desig-equal parent successor)) ()
          "Cannot equate designators that are already equal.")
  (cram-utilities::on-equate-designators successor parent)
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

(defmethod desig-equal ((desig-1 t) (desig-2 t))
  nil)

(defmethod reference :around ((designator designator) &optional role)
  (declare (ignore role))
  (with-slots (lock) designator
    (sb-thread:with-mutex (lock)
      (call-next-method))))

(defmethod reference :after ((desig designator) &optional role)
  (declare (ignore role))
  (with-slots (effective timestamp) desig
    (setf (slot-value desig 'effective) t)
    (unless timestamp
      (setf timestamp (current-timestamp)))))

(defmacro register-designator-class (type class-name)
  "Registers a class as a designator class so that it can be used
together with MAKE-DESIGNATOR and WITH-DESIGNATORS. `type' is the
designator type, e.g. OBJECT and `class-name' is the name of the CLOS
class (derived from class DESIGNATOR), e.g. OBJECT-DESIGNATOR."
  `(setf (get 'make-designator :desig-types)
         (cons (cons ',type ',class-name)
               (remove ',type (get 'make-designator :desig-types) :key #'car))))

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

(defmethod initialize-instance :after ((designator designator) &key)
  (setf (gethash designator *designators*) t))

(defmethod first-desig ((desig designator))
  (if (null (parent desig))
      desig
      (first-desig (parent desig))))

(defmethod current-desig ((desig designator))
  "gets the last known successor in chain, being the most current"
  (if (null (successor desig))
      desig
      (current-desig (successor desig))))
(defmethod current-desig ((desig null))
  "Allow asking current-desig also on NULL objects."
  NIL)

(defun newest-effective-designator (desig)
  (labels ((find-effective-desig (desig)
             (cond ((not desig) nil)
                   ((effective desig)
                    desig)
                   (t (find-effective-desig (parent desig))))))
    (find-effective-desig (current-desig desig))))

(defun make-effective-designator (parent &key new-properties data-object
                                           (timestamp (cut:current-timestamp)))
  "Returns a new designator that has the same type as `parent'. If
`new-properties' is NIL, uses the properties of `parent'. The DATA
slot is bound to `data-object'. It sets the slot EFFECTIVE to T and
returns the newly created designator that is not equated yet."
  (let ((new-designator (make-designator
                         (class-of parent)
                         (or new-properties (properties parent)))))
    (with-slots (data effective
                 (designator-timestamp timestamp))
        new-designator
      (setf data data-object)
      (setf designator-timestamp timestamp)
      (setf effective t))
    new-designator))

(defun designator-solutions (desig &optional from-root)
  "Returns the lazy list of designator references that provide a
  solution for `desig'. If `from-root' is non-nil, the list of all
  solutions beginning from with the original designator is returned.
  Otherwise, the first solution is DESIG's reference."
  (let ((desig (if from-root
                   (first-desig desig)
                   desig)))
    (lazy-list ((current-designator-generator (lambda () desig)))
      (handler-case
          (let ((current-designator (funcall current-designator-generator)))
            (when current-designator
              (handler-case
                  (cont (reference current-designator)
                        (lambda () (next-solution current-designator)))
                (designator-error ()
                  nil))))
        (designator-error ()
          nil)))))

(defun check-desig-prop-package (prop)
  "Checks if `prop' is in the correct package and can be used as a designator property"
  (unless (or (not (symbol-package prop))
              (keywordp prop)
              (eq (symbol-package prop)
                  (symbol-package (intern (symbol-name prop) (find-package :desig-props)))))
    (warn 'simple-warning
          :format-control "Designator property ~s has not been declared properly. This may cause problems. To fix it, use the macro DEF-DESIG-PACKAGE and declare the symbol as a designator property."
          :format-arguments (list prop))))

(defun update-designator-properties (new-properties old-properties)
  "Adds (or replaces) all properties in `new-properties' to
  `old-properties' and returns the new sequence. All properties are of
  the form ([(key value)]*)."
  (reduce (lambda (properties new-property)
            (cons new-property
                  (remove (car new-property) properties
                          :key #'car)))
          new-properties :initial-value old-properties))

(defun copy-designator (old-designator &key (new-description nil))
  "Returns a new (unequated) designator with the same properties as
`old-designator'. When present, the description parameter
`new-description' will be merged with the old description. The new
description will be dominant in this relation."
  (make-designator (get-desig-class old-designator)
                   (merge-desig-descriptions old-designator new-description)))

(defun merge-desig-descriptions (designator new-description)
  "Returns the merge of the description of `designator' with `new-description'.
 The new description will be dominant in this relation."
  (reduce (rcurry (flip #'adjoin) :key #'car)
          (description designator) :initial-value new-description))

(defun extend-designator-properties (designator property-extension)
  "Extends (without merging) the properties of `designator'
by `property-extension' and returns a new (unequated) designator."
  (make-designator
   (class-of designator)
   (append (properties designator) property-extension)))
