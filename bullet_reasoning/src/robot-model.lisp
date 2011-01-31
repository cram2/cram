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

(defgeneric urdf-make-collision-shape (geometry &optional color))

(defmethod urdf-make-collision-shape ((box cl-urdf:box) &optional (color '(0.8 0.8 0.8 1.0)))
  (make-instance 'colored-box-shape
                 :half-extents (cl-transforms:v*
                                (cl-urdf:size box) 0.5)
                 :color color))

(defmethod urdf-make-collision-shape ((cylinder cl-urdf:cylinder) &optional (color '(0.8 0.8 0.8 1.0)))
  (make-instance 'cylinder-shape
                 :half-extents (cl-transforms:make-3d-vector
                                (cl-urdf:radius cylinder)
                                (cl-urdf:radius cylinder)
                                (* 0.5 (cl-urdf:cylinder-length cylinder)))
                 :color color))

(defmethod urdf-make-collision-shape ((sphere cl-urdf:sphere) &optional (color '(0.8 0.8 0.8 1.0)))
  (make-instance 'sphere-shape :radius (cl-urdf:radius sphere)
                 :color color))

(defmethod urdf-make-collision-shape ((mesh cl-urdf:mesh) &optional (color '(0.8 0.8 0.8 1.0)))
  (make-instance 'mesh-shape
                 :color color
                 :faces (physics-utils:3d-model-faces (cl-urdf:3d-model mesh))
                 :points (physics-utils:3d-model-vertices (cl-urdf:3d-model mesh))))

(defclass robot-object (object)
  ((links :initform (make-hash-table :test 'equal) :reader links)
   (joint-states :initform (make-hash-table :test 'equal) :reader joint-states)
   (urdf :initarg :urdf :reader urdf)))

(defgeneric joint-state (robot-object name)
  (:documentation "Returns the value of the joint named `name'"))

(defgeneric (setf joint-state) (new-value robot-object name)
  (:documentation "Sets the specific joint to a new value and updates
  all child-link positions"))

(defgeneric link-pose (robot-object name)
  (:documentation "Gets the pose of a link"))

(defgeneric (setf link-pose) (new-value robot-object name)
  (:documentation "Sets the pose of a link and all its children"))

(defmethod add-object ((world bt-world) (type (eql 'urdf)) name pose &key
                       urdf)
  (labels ((make-link-bodies (pose link)
             "Returns the list of rigid bodies of `link' and all its sub-links"
             (let* ((pose-transform (cl-transforms:reference-transform pose))
                    (collision-elem (cl-urdf:collision link))
                    (bodies (mapcan (lambda (joint)
                                      (make-link-bodies (cl-transforms:transform-pose
                                                         pose-transform (cl-urdf:origin joint))
                                                        (cl-urdf:child joint)))
                                    (cl-urdf:to-joints link))))
               (if collision-elem
                   (cons (cons
                          (cl-urdf:name link)
                          (make-instance
                           'rigid-body
                           :name (make-rigid-body-name name (cl-urdf:name link))
                           :mass 0
                           :pose (cl-transforms:transform-pose
                                  pose-transform (cl-urdf:origin collision-elem))
                           :collision-shape (urdf-make-collision-shape
                                             (cl-urdf:geometry collision-elem)
                                             (cl-urdf:color (cl-urdf:material (cl-urdf:visual link))))
                           :collision-flags '(:cf-default)))
                         bodies)
                   bodies))))
    (let* ((urdf-model (etypecase urdf
                         (cl-urdf:robot urdf)
                         (string (handler-bind ((cl-urdf:urdf-type-not-supported #'muffle-warning))
                                   (cl-urdf:parse-urdf urdf)))))
           (pose (ensure-pose pose))
           (bodies (make-link-bodies pose (cl-urdf:root-link urdf-model)))
           (object
            (make-instance 'robot-object
                           :rigid-bodies (mapcar #'cdr bodies)
                           :world world
                           :pose-reference-body (cl-urdf:name (cl-urdf:root-link urdf-model))
                           :name name
                           :urdf urdf-model
                           :group :character-filter
                           :mask '(:default-filter :static-filter))))
      (loop for (name . body) in bodies do
            (setf (gethash name (slot-value object 'links))
                  body))
      (loop for name being the hash-keys in (cl-urdf:joints urdf-model) do
        (setf (gethash name (joint-states object)) 0.0d0))
      object)))

(defun update-link-poses (robot-object link pose)
  "Updates the pose of `link' and all its children according to
current joint states"
  (let ((links (links robot-object))
        (joint-states (joint-states robot-object)))
    (let ((body (gethash (cl-urdf:name link) links))
          (pose-transform (cl-transforms:reference-transform pose)))
      (when body
        (setf (pose body) (cl-transforms:transform-pose
                           pose-transform
                           (cl-urdf:origin (cl-urdf:collision link)))))
      (dolist (to-joint (cl-urdf:to-joints link))
        (update-link-poses robot-object
                           (cl-urdf:child to-joint)
                           (cl-transforms:transform-pose
                            pose-transform
                            (cl-transforms:transform-pose
                             (cl-urdf:origin to-joint)
                             (joint-transform to-joint (gethash
                                                        (cl-urdf:name to-joint)
                                                        joint-states)))))))))

(defun joint-transform (joint value)
  (case (cl-urdf:joint-type joint)
    ((:revolute :continuous)
       (cl-transforms:make-transform
        (cl-transforms:make-3d-vector 0 0 0)
        (cl-transforms:axis-angle->quaternion
         (cl-urdf:axis joint)
         value)))
    (:prismatic
       (cl-transforms:make-transform
        (cl-transforms:v* (cl-urdf:axis joint) value)
        (cl-transforms:make-quaternion 0 0 0 1)))
    (t (cl-transforms:make-transform
        (cl-transforms:make-3d-vector 0 0 0)
        (cl-transforms:make-quaternion 0 0 0 1)))))

(defun calculate-joint-state (obj name)
  (with-slots (urdf) obj
    (let ((links (links obj))
          (joint (gethash name (cl-urdf:joints urdf))))
      (when joint
        (let* ((parent (cl-urdf:parent joint))
               (parent-body (gethash (cl-urdf:name parent) links))
               (child (cl-urdf:child joint))
               (child-body (gethash (cl-urdf:name child) links)))
          (when (and parent-body child-body)
            (let ((origin (cl-transforms:transform*
                           (cl-transforms:reference-transform
                            (pose parent-body))
                           (cl-transforms:transform-inv
                            (cl-transforms:reference-transform
                             (cl-urdf:origin (cl-urdf:collision parent))))
                           (cl-urdf:origin joint)))
                  (child-transform (cl-transforms:transform*
                                    (cl-transforms:reference-transform
                                     (pose child-body))
                                    (cl-transforms:transform-inv
                                     (cl-transforms:reference-transform
                                      (cl-urdf:origin (cl-urdf:collision child)))))))
              (case (cl-urdf:joint-type joint)
                ((:revolute :continuous)
                   (multiple-value-bind (angle axis)
                       (cl-transforms:angle-between-quaternions
                        (cl-transforms:rotation origin)
                        (cl-transforms:rotation child-transform))
                     (if (< (cl-transforms:dot-product
                             axis (cl-urdf:axis joint))
                            0)
                         (* angle -1)
                         angle)))
                (:prismatic
                   (* (cl-transforms:v-dist
                       (cl-transforms:translation origin)
                       (cl-transforms:translation child-transform))
                      (if (< (cl-transforms:dot-product
                              (cl-transforms:v-
                               (cl-transforms:translation child-transform)
                               (cl-transforms:translation origin))
                              (cl-urdf:axis joint))
                             0)
                          -1 1)))
                (t 0.0d0)))))))))

(defmethod invalidate-object :after ((obj robot-object))
  (with-slots (world links joint-states) obj
    (loop for name being the hash-keys in links
          using (hash-value body) do
            (setf (gethash name links)
                  (rigid-body obj (name body))))
    (loop for name being the hash-keys in joint-states do
      (setf (gethash name joint-states) (or (calculate-joint-state obj name) 0.0d0)))))

(defmethod joint-state ((obj robot-object) name)
  (nth-value 0 (gethash name (joint-states obj))))

(defmethod (setf joint-state) (new-value (obj robot-object) name)
  (with-slots (urdf) obj
    (let* ((links (links obj))
           (joint-states (joint-states obj))
           (joint (gethash name (cl-urdf:joints urdf)))
           (parent (cl-urdf:parent joint))
           (parent-body (gethash (cl-urdf:name parent) links)))
      (when (and joint parent-body)
        (let ((joint-transform
               (cl-transforms:transform*
                (cl-transforms:reference-transform
                 (cl-urdf:origin joint))
                (joint-transform joint new-value))))
          (setf (gethash name joint-states) new-value)
          (update-link-poses
           obj (cl-urdf:child joint)
           (cl-transforms:transform*
            (cl-transforms:reference-transform
             (pose parent-body))
            (cl-transforms:transform-inv
             (cl-transforms:reference-transform
              (cl-urdf:origin (cl-urdf:collision parent))))
            joint-transform)))))))

(defmethod link-pose ((obj robot-object) name)
  ;; We need to handle two different cases here. One is when we have a
  ;; rigid body for a specific link. Then reading the pose is just
  ;; returning the pose of the body minus the pose of the collision
  ;; object in the body. The second case is a link without a rigid
  ;; body, i.e. without a collision model. We need to walk up the tree
  ;; until we reach a body, collecting the transforms along that path.
  (with-slots (links urdf) obj
    (let ((link (gethash name (cl-urdf:links urdf)))
          (body (gethash name links)))
      (cond (body
             (cl-transforms:transform->pose
              (cl-transforms:transform*
               (cl-transforms:reference-transform
                (pose body))
               (cl-transforms:transform-inv
                (cl-transforms:reference-transform
                 (cl-urdf:origin (cl-urdf:collision link)))))))
            (t
             (cl-transforms:transform-pose
              (cl-transforms:reference-transform
               (link-pose obj (cl-urdf:name (cl-urdf:parent
                                             (cl-urdf:from-joint link)))))
              (cl-urdf:origin (cl-urdf:from-joint link))))))))

(defmethod (setf link-pose) (new-value (obj robot-object) name)
  (with-slots (urdf) obj
    (let* ((links (links obj))
           (link (gethash name (cl-urdf:links urdf)))
          (body (gethash name links)))
      ;; Should we throw an error here?
      (when body
        (update-link-poses obj link new-value)
        (let ((joint (cl-urdf:from-joint link)))
          (when joint
            (setf (gethash (cl-urdf:name joint) (joint-states obj))
                  (calculate-joint-state obj (cl-urdf:name joint)))))))))

(defmethod pose ((obj robot-object))
  (link-pose obj (slot-value obj 'pose-reference-body)))

(defmethod (setf pose) (new-value (obj robot-object))
  (setf (link-pose obj (slot-value obj 'pose-reference-body)) new-value))
