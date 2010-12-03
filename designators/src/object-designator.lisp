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

(in-package :desig)

(defvar *object-designator-resolvers* nil)

(defstruct object-desig-resolver
  name namespace function)

(defclass object-designator (designator designator-id-mixin)
  ())

(register-designator-type object object-designator)

(defun add-obj-desig-resolver (resolver)
  (let ((prev-resolver (find (object-desig-resolver-name resolver)
                             *object-designator-resolvers*
                             :key #'object-desig-resolver-name)))
    (if prev-resolver
        (setf (object-desig-resolver-namespace prev-resolver)
              (object-desig-resolver-namespace resolver)
              (object-desig-resolver-function prev-resolver)
              (object-desig-resolver-function resolver))
        (push resolver *object-designator-resolvers*))))

(defmacro register-object-desig-resolver (name namespace (prev-param desig-param) &body body)
  "Registers a designator resolver. When converting the designator into something useful,
   i.e. into a format the perception routines can work with. It does
   this by reducing the designator to be resolved over the list of
   *object-designator-resolvers* the match the current designator
   namespace."
  `(add-obj-desig-resolver (make-object-desig-resolver
                            :name ',name :namespace ,namespace
                            :function (lambda (,prev-param ,desig-param)
                                        ,@body))))

(defun resolve-object-desig (desig namespace)
  (let ((info (reduce (lambda (prev resolver)
                        (when (eq (object-desig-resolver-namespace resolver)
                                  namespace)
                          (funcall (object-desig-resolver-function resolver)
                                   prev desig)))
                      *object-designator-resolvers*
                      :initial-value nil)))
    ;; (when (valid desig)
    ;;   (push (object-pose (reference desig))
    ;;         (cop-desig-location-info-poses (cop-desig-info-location info))))
    info))

(defmethod reference ((desig object-designator))
  (or (slot-value desig 'data)
      (error "Designator does not reference an object.")))

;; (defun desig-compatible-descriptions (desc-1 desc-2)
;;   (multiple-value-bind (desc-1 desc-2) (if (< (list-length desc-1)
;;                                               (list-length desc-2))
;;                                            (values desc-1 desc-2)
;;                                            (values desc-2 desc-1))
;;     (loop for (prop-1-name prop-1-val) in desc-1
;;        when (let ((prop-2-val (find prop-1-name desc-2 :key #'car)))
;;               (when prop-2-val
;;                 (not (eql prop-1-val prop-2-val))))
;;        do (return nil)
;;        finally (return t))))
