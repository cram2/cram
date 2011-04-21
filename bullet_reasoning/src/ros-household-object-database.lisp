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
;;;

(in-package :btr)

(defvar *ros-object-database-models* (make-hash-table :test 'equal))
(defvar *ros-object-database-model-meshes* (make-hash-table))

(defun init-ros-object-database (&optional (set-name "REDUCED_MODEL_SET"))
  (roslisp:with-fields ((code (code return_code))
                        (model-ids model_ids))
      (roslisp:call-service
       "/objects_database_node/get_model_list"
       'household_objects_database_msgs-srv:getmodellist
       :model_set set-name)
    (when (eql code -1)
      (clrhash *ros-object-database-models*)
      (map 'nil
           (lambda (id)
             (map
              'nil
              (lambda (name)
                (setf (gethash name *ros-object-database-models*)
                      id))
              (get-ros-model-names id)))
           model-ids))))

(defun get-ros-model-names (id)
  (roslisp:with-fields ((code (code return_code))
                        (tags tags))
      (roslisp:call-service
       "/objects_database_node/get_model_description"
       'household_objects_database_msgs-srv:getmodeldescription
       :model_id id)
    (when (eql code -1)
      tags)))

(defun get-ros-model-mesh (model-id)
  (let ((id (etypecase model-id
              (number model-id)
              (string (gethash model-id *ros-object-database-models*)))))
    (or (gethash id *ros-object-database-model-meshes*)
        (roslisp:with-fields ((code (code return_code))
                              (mesh mesh))
            (roslisp:call-service
             "/objects_database_node/get_model_mesh"
             'household_objects_database_msgs-srv:getmodelmesh
             :model_id id)
          (when (eql code -1)
            (setf (gethash id *ros-object-database-model-meshes*)
                  (physics-utils:shape-msg->mesh mesh :disable-type-check t)))))))

(defmethod add-object ((world bt-world) (type (eql 'ros-household-object)) name pose &key
                       mass mesh-id (color '(0.8 0.8 0.8 1.0)))
  (let ((mesh (get-ros-model-mesh mesh-id)))
    (assert mesh)
    (make-object world name
                 (list
                  (make-instance
                   'rigid-body
                   :name name :mass mass :pose (ensure-pose pose)
                   :collision-shape (make-instance
                                     'convex-hull-mesh-shape
                                     :points (physics-utils:3d-model-vertices mesh)
                                     :faces (physics-utils:3d-model-faces mesh)
                                     :color color
                                     :disable-face-culling t))))))