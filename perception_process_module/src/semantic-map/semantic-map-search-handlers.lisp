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

(defclass semantic-map-perceived-object (perceived-object)
  ((semantic-map-object :initarg :semantic-map-object
                        :reader semantic-map-object))
  (:default-initargs :probability 1.0))

(defgeneric owl-name (sem-map-po)
  (:method ((obj semantic-map-perceived-object))
    (with-slots (semantic-map-object) obj
      (sem-map-utils:owl-name semantic-map-object))))

(defmethod initialize-instance :after ((obj semantic-map-perceived-object) &key)
  (with-slots (semantic-map-object pose) obj
    (setf pose (sem-map-utils:pose semantic-map-object))))

(defmethod make-new-desig-description ((old-desig object-designator)
                                       (po semantic-map-perceived-object))
  (let ((description (call-next-method)))
    (if (member 'name description :key #'car)
        description
        (cons `(name ,(name po)) description))))

;; The generic function of this symbol is defined in
;; semantic_map_utils already.
(defmethod name ((obj semantic-map-perceived-object))
  (with-slots (semantic-map-object) obj
    (sem-map-utils:name semantic-map-object)))

(defun resolve-sem-map-obj-desig (desig)
  (mapcar (lambda (sem-map-obj)
            (make-instance 'semantic-map-perceived-object
              :semantic-map-object sem-map-obj))
          (sem-map-utils:designator->semantic-map-objects desig)))

(def-object-search-function query-semantic-map-object-name semantic-map
    (((name ?name)) desig perceived-object)
  (declare (ignore perceived-object))
  (resolve-sem-map-obj-desig desig))

(def-object-search-function query-semantic-map-object-type semantic-map
    (((type ?type)) desig perceived-object)
  (declare (ignore perceived-object))
  (resolve-sem-map-obj-desig desig))

(def-object-search-function query-semantic-map-object-part semantic-map
    (((part-of ?parent)) desig perceived-object)
  (declare (ignore perceived-object))
  (resolve-sem-map-obj-desig desig))
