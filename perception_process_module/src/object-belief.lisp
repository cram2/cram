;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :perception-pm)

(defgeneric object-pose (obj)
  (:documentation "Returns the pose of a designator reference."))

(defgeneric object-properties (obj)
  (:documentation "Returns a valid designator description of the
                   perceived object, i.e. a set of tuples."))

(defgeneric object-desig (obj)
  (:documentation "Returns the designator that references this object
  instance."))

(defgeneric (setf object-desig) (new-val obj)
  (:documentation "Returns the designator that references this object
  instance."))

(defgeneric object-timestamp (obj)
  (:documentation "Returns the timestamp this object has been
  detected."))

(defgeneric make-new-desig-description (old-desig perceived-object)
  (:documentation "Merges the description of `old-desig' with the
  properties of `perceived-object'"))

(defclass perceived-object ()
  ((pose :accessor object-pose :initarg :pose)
   (probability :accessor perceived-object-probability :initarg :probability)
   (desig :accessor object-desig :initarg :desig :initform nil)
   (timestamp :accessor object-timestamp :initarg :timestamp
              :initform (current-timestamp))))

(defclass queried-object ()
  ((pose :accessor object-pose :initarg :pose)
   (cop-id :accessor object-id :initarg :cop-id)
   (desig :accessor object-desig :initarg :desig :initform nil)
   (timestamp :accessor object-timestamp :initarg :timestamp
              :initform (current-timestamp))
   (object-properties :accessor object-properties :initarg :object-properties)))

(defgeneric matching-object (object candidates)
  (:documentation "This generic function is a hook. Every hook
  function gets an `object' and a list of `candidates' and returns a
  list of the form `(object . score) with object being an object out
  of `canditates' that matches `object' best and score indicating the
  value of the match. Higher scores indicate a better match. The
  result of executing MATCHING-OBJECT is then the object with the
  highest score.")
  (:method-combination hooks
                       :hook-combination (lambda (&rest results)
                                           (loop for r in results
                                              with max = (car results)
                                              when (and (cdr r)
                                                        (cdr max)
                                                        (> (cdr r) (cdr max)))
                                              do (setq max r)
                                              finally (return (car max))))))

(defun designator->production (desig var-name)
  (loop for prop in (description desig)
        unless (eq (car prop) 'at)
          collecting `(,(car prop) ,var-name ,@(cdr prop))))

(defun assert-perceived-object (perceived-object properties)
  ;; For now, properties is a desig description. Not sure if this
  ;; makes sense, but we will see.  TODO: read perceived-object
  ;; properties and do additional assertions.
  (loop for prop in properties
     do (rete-assert `(,(car prop) ,perceived-object ,@(cdr prop)))))

(defun retract-perceived-object (perceived-object)
  "Retracts all facts the were asserted for `perceived-object'"
  (let ((bdgs (rete-holds `(?lhs ,perceived-object . ?rhs))))
    (loop for bdg in bdgs do
         (with-vars-bound (?lhs ?rhs)
             bdg
           (rete-retract `(,?lhs ,perceived-object ,?rhs))))))

(defun compatible-properties (props-1 props-2)
  (or (null props-1)
      (let ((prop-1-value (cadar props-1))
            (prop-2-value (cadr (assoc (caar props-1) props-2))))
        (when (or (null prop-2-value)
                  (eq prop-1-value prop-2-value)
                  (or
                   (when (json-prolog:wait-for-prolog-service 0.5)
                     (json-prolog:prolog-1
                      `(cop-compatible-results ,prop-1-value ,prop-2-value)))
                   (prolog `(obj-subtype ,prop-1-value ,prop-2-value))
                   (prolog `(obj-subtype ,prop-2-value ,prop-1-value))))
          (compatible-properties (cdr props-1) props-2)))))

(defun merge-desig-descriptions (old-description new-description)
  "Merges the designator descriptions `old-description' and
   `new-description'. The result is a union of the two
   descriptions. For all properties that occur in both descriptions
   but have a different property value, the property of
   `new-description' is used. "
  (reduce (rcurry (flip #'adjoin) :key #'car)
          old-description :initial-value new-description))

(defun desig-current-perceived-object (desig &optional (type 'perceived-object))
  (labels ((doit (d)
             (cond ((typep (slot-value d 'data) type)
                    (reference d))
                   ((parent d)
                    (doit (parent d))))))
    (doit (current-desig desig))))

(defmethod make-new-desig-description ((old-desig object-designator) (qo queried-object))
  (merge-desig-descriptions (description old-desig) (object-properties qo)))

(defmethod matching-object :euclidean-distance (object candidates)
  (labels ((as-pose-stamped (obj)
             (typecase obj
               (jlo:jlo (jlo->pose obj))
               (tf:stamped-transform
                  (tf:make-pose-stamped
                   (tf:frame-id obj)
                   (tf:stamp obj)
                   (cl-transforms:translation obj)
                   (cl-transforms:rotation obj)))
               (cl-transforms:transform
                  (tf:make-pose-stamped
                   "/map" (ros-time)
                   (cl-transforms:translation obj)
                   (cl-transforms:rotation obj)))
               (cl-transforms::pose
                  (tf:make-pose-stamped
                   "/map" (ros-time)
                   (cl-transforms:origin obj)
                   (cl-transforms:orientation obj)))
               (t obj)))
           (distance (obj-1 obj-2)
             (let ((pose-1 (as-pose-stamped obj-1))
                   (pose-2 (as-pose-stamped obj-2)))
               (cl-transforms:v-dist (cl-transforms:origin pose-1)
                                     (cl-transforms:origin pose-2)))))
    (labels ((closest-object (o seq &optional curr)
               (cond ((null seq)
                      (values (car curr) (cdr curr)))
                     (t
                      (closest-object
                       o (cdr seq)
                       (let ((dist (distance (object-pose o) (object-pose (car seq))) ))
                         (if (or (not curr) (< dist (cdr curr)))
                             (cons (car seq) dist)
                             curr)))))))
      (when candidates
        (multiple-value-bind (obj dist)
            (closest-object object candidates)
          ;; TODO: Use some customizable constant here
          (when (< dist 0.5)
            (cons obj 1)))))))
