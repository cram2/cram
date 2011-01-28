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
  ((links :initform (make-hash-table :test 'equal))
   (joints :initform (make-hash-table :test 'equal))
   (urdf :initarg :urdf :reader urdf)))

(defmethod add-object ((world bt-world) (type (eql 'urdf)) name pose &key
                       urdf)
  (labels ((make-link-bodies (pose link)
             "Returns the list of rigid bodies of `link' and all its sub-links"
             (let ((pose-transform (cl-transforms:reference-transform pose))
                   (collision-elem (cl-urdf:collision link)))
               (if collision-elem
                   (cons (cons
                          (cl-urdf:name link)
                          (make-instance
                           'rigid-body
                           :name (make-rigid-body-name name (cl-urdf:name link))
                           :pose (cl-transforms:transform-pose
                                  pose-transform (cl-urdf:origin collision-elem))
                           :collision-shape (urdf-make-collision-shape
                                             (cl-urdf:geometry collision-elem)
                                             (cl-urdf:color (cl-urdf:material (cl-urdf:visual link))))
                           :collision-flags '(:cf-default)))
                         (mapcan (lambda (joint)
                                   (make-link-bodies (cl-transforms:transform-pose
                                                      pose-transform (cl-urdf:origin joint))
                                                     (cl-urdf:child joint)))
                                 (cl-urdf:to-joints link)))
                   (mapcan (lambda (joint)
                             (make-link-bodies (cl-transforms:transform-pose
                                                pose-transform (cl-urdf:origin joint))
                                               (cl-urdf:child joint)))
                           (cl-urdf:to-joints link))))))
    (let* ((urdf-model (etypecase urdf
                         (cl-urdf:robot urdf)
                         (string (handler-bind ((cl-urdf:urdf-type-not-supported #'muffle-warning))
                                   (cl-urdf:parse-urdf urdf)))))
           (pose (ensure-pose pose))
           (bodies (make-link-bodies pose (cl-urdf:root-link urdf-model))))
      (make-instance 'robot-object
                     :rigid-bodies (mapcar #'cdr bodies)
                     :world world
                     :pose-reference-body (cl-urdf:name (cl-urdf:root-link urdf-model))
                     :name name
                     :urdf urdf-model
                     :group :character-filter
                     :mask '(:default-filter :static-filter)))))
