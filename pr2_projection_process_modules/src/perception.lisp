;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :projection-process-modules)

(defclass projection-object-designator (desig:object-designator)
  ())

(defclass perceived-object (desig:object-designator-data)
  ((designator :reader object-designator :initarg :designator)))

(defmethod desig:designator-pose ((designator projection-object-designator))
  (desig:object-pose (desig:reference designator)))

(defmethod desig:designator-distance ((designator-1 desig:object-designator)
                                      (designator-2 desig:object-designator))
  (cl-transforms:v-dist (cl-transforms:origin (desig:designator-pose designator-1))
                        (cl-transforms:origin (desig:designator-pose designator-2))))

(defun make-object-designator (perceived-object &key parent type name)
  (let* ((pose (desig:object-pose perceived-object))
         (designator (change-class
                      (desig:make-designator
                       'desig-props:object
                       (desig:update-designator-properties
                        `(,@(when type `((desig-props:type ,type)))
                            (desig-props:at ,(desig:make-designator
                                              'desig:location `((desig-props:pose ,pose))))
                            ,@(when name `((desig-props:name ,name))))
                        (when parent (desig:properties parent)))
                       parent)
                      'projection-object-designator)))
    (setf (slot-value perceived-object 'designator) designator)
    (setf (slot-value designator 'desig:data) perceived-object)
    (setf (slot-value designator 'desig:valid) t)
    designator))

(defun find-object (designator)
  "Finds objects with (optional) name `object-name' and type `type'
  and returns a list of elements of the form \(name pose\)."
  (let ((object-name (when (slot-value designator 'desig:data)
                       (desig:object-identifier (desig:reference designator))))
        (type (or (desig:desig-prop-value designator 'desig-props:type)
                  '?_)))
    (flet ((find-household-object ()
             (cut:force-ll
              (cut:lazy-mapcar
               (lambda (solution)
                 (cut:with-vars-strictly-bound (?object ?pose) solution
                   (list ?object ?pose)))
               (crs:prolog `(and (robot ?robot)
                                 (bullet-world ?world)
                                 ,@(when object-name
                                     `((crs:== ?object ,object-name)))
                                 (object ?world ?object)
                                 (household-object-type ?world ?object ,type)
                                 (visible ?world ?robot ?object)
                                 (pose ?world ?object ?pose))))))
           (find-handle ()
             (mapcar (lambda (semantic-map-object)
                       (list
                        (sem-map-utils:name semantic-map-object)
                        (sem-map-utils:pose semantic-map-object)))
                     (sem-map-utils:designator->semantic-map-objects designator))))
      (case type
        (desig-props:handle (find-handle))
        (t (find-household-object))))))

(defun find-with-bound-designator (designator)
  (flet ((make-designator (object pose)
           (make-object-designator
            (make-instance 'perceived-object
              :object-identifier object
              :pose pose)
            :name object
            :parent designator)))
    (cut:force-ll
     (cut:lazy-mapcar
      (alexandria:curry #'apply #'make-designator) (find-object designator)))))

(defun find-with-new-designator (designator)
  (desig:with-desig-props (desig-props:type) designator
    (flet ((make-designator (object pose)
             (make-object-designator
              (make-instance 'perceived-object
                :object-identifier object
                :pose pose)
              :type desig-props:type
              :name object)))
      (when desig-props:type
        (cut:force-ll
         (cut:lazy-mapcar
          (alexandria:curry #'apply #'make-designator) (find-object designator)))))))

(def-process-module projection-perception (input)
  (let ((newest-valid-designator (desig:newest-valid-designator input)))
    (or
     (mapcar (lambda (designator)
               (cram-plan-knowledge:on-event
                (make-instance 'cram-plan-knowledge:object-perceived-event
                  :perception-source :projection
                  :object-designator designator))
               designator)
             (if newest-valid-designator
                 (find-with-bound-designator newest-valid-designator)
                 (find-with-new-designator input)))
     (cpl:fail 'cram-plan-failures:object-not-found
               :object-desig input))))
