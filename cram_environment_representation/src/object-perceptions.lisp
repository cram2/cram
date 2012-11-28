;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-environment-representation)

(cpl:define-task-variable *object-identifier-to-instance-mappings*
    (make-hash-table :test #'equal)
    "Mapping from object-identifiers as bound in the
OBJECT-DESIGNATOR-DATA class to instance names in the bullet world
database.")

(cram-projection:define-special-projection-variable
    *object-identifier-to-instance-mappings*
    (alexandria:copy-hash-table *object-identifier-to-instance-mappings*))

(defmethod clear-belief object-identifiers ()
  (clrhash *object-identifier-to-instance-mappings*))

(defgeneric register-object-designator-data (data &key type)
  (:documentation "Registers an object in the belief state. The object
is identified by its corresponding OBJECT-DESIGNATOR-DATA object. This
method is also responsible for entity resolution, i.e. for finding the
right object instance that needs to be updated and for adding new
instances if necessary.

The default implementation for the objects defined in
CRAM-MANIPULATION-KNOWLEDGE completely trust the object identifiers,
i.e. if an object instance for an identifier has been registered
already, the pose of the corresponding object in the world database is
just updated. Otherwise a new instance is created."))

(defgeneric object-mass (data)
  (:documentation "Returns the (maybe approximated) mass of an object
  identified by `data' which is of type OBJECT-DESIGNATOR-DATA."))

(defmethod register-object-designator-data
    ((data cram-manipulation-knowledge:object-shape-data-mixin) &key type)
  (declare (ignore type))
  nil)

(defmethod register-object-designator-data
    ((data cram-manipulation-knowledge:object-mesh-data-mixin) &key type)
  (let ((instance-name (or
                        (gethash (desig:object-identifier data)
                                 *object-identifier-to-instance-mappings*)
                        (setf (gethash (desig:object-identifier data)
                                       *object-identifier-to-instance-mappings*)
                              (gensym (string (desig:object-identifier data)))))))
    (prolog `(and (bullet-world ?world)
                  (-> (object ?world ,instance-name)
                      (assert ?world (btr:object-pose
                                      ,instance-name
                                      ,(desig:object-pose data)))
                      (assert ?world (btr:object btr:mesh ,instance-name
                                                 ,(desig:object-pose data)
                                                 :mesh ,(object-mesh data)
                                                 :mass ,(object-mass data)
                                                 :types ,(list type)
                                                 :disable-face-culling t)))))))

(defmethod register-object-designator-data
    ((data cram-manipulation-knowledge:object-point-data-mixin) &key type)
  (declare (ignore type))
  nil)

(defmethod object-mass ((data cram-manipulation-knowledge:object-mesh-data-mixin))
  (physics-utils:calculate-mass
   :mesh :points (cram-manipulation-knowledge:vertices data)))

(defun object-mesh (data)
  (declare (type cram-manipulation-knowledge:object-mesh-data-mixin data))
  (with-slots ((vertices cram-manipulation-knowledge:vertices)
               (faces cram-manipulation-knowledge:faces)) data
    (physics-utils:make-3d-model
     :vertices vertices
     :faces (physics-utils:fix-normals
             (map 'vector (lambda (indices)
                            (physics-utils:make-face
                             :points (mapcar (alexandria:curry #'elt vertices) indices)))
                  faces)
             :always-recalculate t))))
