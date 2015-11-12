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

(defparameter *mesh-files*
  '((:cap "package://cram_bullet_reasoning/resource/cap.stl" t)
    (:glasses "package://cram_bullet_reasoning/resource/glasses.stl" nil)
    (:glove "package://cram_bullet_reasoning/resource/glove.stl" nil)
    (:shoe "package://cram_bullet_reasoning/resource/shoe.stl" nil)))

(defclass manmade-object (object)
  ((types :reader manmade-object-types :initarg :types)))

(defmethod copy-object ((object manmade-object) (world bt-reasoning-world))
  (change-class (call-next-method) 'manmade-object
                :types (manmade-object-types object)))

(defgeneric manmade-object-dimensions (object)
  (:method ((object manmade-object))
    (bounding-box-dimensions (aabb object)))
  (:method ((object-type symbol))
        (let ((mesh-specification (assoc object-type *mesh-files*)))
          (assert
           mesh-specification ()
           "Couldn't fine a mesh for object type ~a." object-type)
          (destructuring-bind (type uri &optional flip-winding-order)
              mesh-specification
            (declare (ignore type))
            (let ((model-filename (physics-utils:parse-uri uri)))
              (with-file-cache
                  model model-filename (physics-utils:load-3d-model
                                        model-filename
                                        :flip-winding-order flip-winding-order)
                (values
                 (physics-utils:calculate-aabb
                  (physics-utils:3d-model-vertices model)))))))))


(defun make-manmade-object (world name types &optional bodies (add-to-world t))
  (make-instance 'manmade-object
    :name name
    :world world
    :rigid-bodies bodies
    :add add-to-world
    :types types))

(defmethod add-object ((world bt-world) (type (eql :mesh)) name pose
                       &key mass mesh (color '(1.0 1.0 1.0 1.0)) types (scale 1.0)
                         disable-face-culling)
  (let ((mesh-model (physics-utils:scale-3d-model
                     (etypecase mesh
                       (symbol (let ((uri (physics-utils:parse-uri
                                           (cadr (assoc mesh *mesh-files*)))))
                                 (with-file-cache model uri
                                     (physics-utils:load-3d-model
                                      uri :flip-winding-order (caddr (assoc mesh *mesh-files*)))
                                   model)))
                       (string (let ((uri  (physics-utils:parse-uri mesh)))
                                 (with-file-cache model uri (physics-utils:load-3d-model uri)
                                   model)))
                       (physics-utils:3d-model mesh))
                     scale)))
    (make-manmade-object world name (or types (list mesh))
                           (list
                            (make-instance 'rigid-body
                              :name name :mass mass :pose (ensure-pose pose)
                              :collision-shape
                              (make-instance 'convex-hull-mesh-shape
                                :points (physics-utils:3d-model-vertices mesh-model)
                                :faces (physics-utils:3d-model-faces mesh-model)
                                :color color
                                :disable-face-culling disable-face-culling))))))

