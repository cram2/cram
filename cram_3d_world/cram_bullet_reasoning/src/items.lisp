;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
  '((:mug "package://cram_bullet_reasoning/resource/mug.stl" t)
    (:plate "package://cram_bullet_reasoning/resource/plate.stl" nil)
    (:mondamin "package://cram_bullet_reasoning/resource/mondamin.stl" nil)
    (:pot "package://cram_bullet_reasoning/resource/pot-ww.stl" nil)
    (:weisswurst "package://cram_bullet_reasoning/resource/ww.stl" nil)
    (:bowl "package://cram_bullet_reasoning/resource/bowl.stl" nil)
    (:fork "package://cram_bullet_reasoning/resource/fork.stl" nil)
    (:knife "package://cram_bullet_reasoning/resource/knife.stl" nil)
    (:spatula "package://cram_bullet_reasoning/resource/spatula.stl" nil)
    (:cap "package://cram_bullet_reasoning/resource/cap.stl" t)
    (:glasses "package://cram_bullet_reasoning/resource/glasses.stl" nil)
    (:glove "package://cram_bullet_reasoning/resource/glove.stl" nil)
    (:shoe "package://cram_bullet_reasoning/resource/shoe.stl" nil)))

(defun add-objects-to-mesh-list (ros-package &key (directory "resource") extension)
  "Adds all meshes from `ros-package' resource directory into *mesh-files* list.
The name in the list is a keyword that is created by lispifying the filename."
  (mapcar (lambda (object-filename-and-object-extension)
            (declare (type list object-filename-and-object-extension))
            (destructuring-bind (object-filename object-extension)
                object-filename-and-object-extension
              (if (if extension
                      (string-equal object-extension extension)
                      (or (string-equal object-extension "stl")
                          (string-equal object-extension "dae")))
                  (let* ((lisp-name (roslisp-utilities:lispify-ros-underscore-name
                                    object-filename :keyword))
                         (new-entry (list lisp-name
                                          (format nil "package://~a/~a/~a.~a"
                                                  ros-package directory
                                                  object-filename object-extension)
                                          nil))
                         (position-of-entry (position lisp-name *mesh-files* :key #'car)))
                    (if position-of-entry
                        (setf (nth position-of-entry *mesh-files*) new-entry)
                        (push new-entry *mesh-files*))
                    lisp-name))))
          (mapcar (lambda (pathname)
                    (list (pathname-name pathname) (pathname-type pathname)))
                  (directory (physics-utils:parse-uri
                              (format nil "package://~a/~a/*.*" ros-package directory))))))

(defclass item (object)
  ((types :reader item-types :initarg :types)))

(defmethod copy-object ((object item) (world bt-reasoning-world))
  (change-class (call-next-method) 'item
                :types (item-types object)))

(defgeneric item-dimensions (object)
  (:method ((object item))
    (bounding-box-dimensions (aabb object)))
  (:method ((object-type symbol))
    (or (cutlery-dimensions object-type)
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
                  (physics-utils:3d-model-vertices model))))))))))

(defgeneric cutlery-dimensions (type)
  (:method ((type t))
    nil)
  (:method ((type (eql :knife)))
    (cl-transforms:make-3d-vector 0.1 0.01 0.005))
  (:method ((type (eql :fork)))
    (cl-transforms:make-3d-vector 0.1 0.015 0.005)))

(defun make-item (world name types &optional bodies (add-to-world t))
  (make-instance 'item
    :name name
    :world world
    :rigid-bodies bodies
    :add add-to-world
    :types types))

(defun make-octagon-prism-shape (radius height)
  "Returns a collision shape that is a octagon prism, i.e. that has an
  octagon at its base."
  (let ((compound-shape (make-instance 'compound-shape)))
    (dotimes (i 4)
      (add-child-shape compound-shape
                       (cl-transforms:make-pose
                        (cl-transforms:make-3d-vector 0 0 0)
                        (cl-transforms:axis-angle->quaternion
                         (cl-transforms:make-3d-vector 0 0 1)
                         (/ (* i pi)
                            4)))
                       (make-instance
                        'box-shape
                        :half-extents (cl-transforms:make-3d-vector
                                       radius (* radius (sin (/ pi 8)))
                                       (/ height 2)))))
    compound-shape))

(defun make-cup-shape (radius height handle-size)
  (let ((collision-shape (make-octagon-prism-shape radius height)))
    (add-child-shape collision-shape
                     (cl-transforms:make-pose
                      (cl-transforms:make-3d-vector
                       (+ (* radius (cos (/ pi 8)))
                          (/ (cl-transforms:x handle-size)
                             2))
                       0 0)
                      (cl-transforms:make-quaternion 0 0 0 1))
                     (make-instance
                      'box-shape
                      :half-extents (cl-transforms:v* handle-size 0.5)))
    collision-shape))

(defmethod add-object ((world bt-world) (type (eql :generic-cup)) name pose &key
                       mass radius height
                       (handle-size (cl-transforms:make-3d-vector
                                     0.03 0.01 (* height 0.8))))
  (make-item world name '(generic-cup)
             (list
              (make-instance
                  'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-cup-shape radius height handle-size)))))

(defmethod add-object ((world bt-world) (type (eql :mug)) name pose &key
                       mass)
  (add-object world :mesh name pose :mass mass :mesh :mug))

(defmethod add-object ((world bt-world) (type (eql :mesh)) name pose
                       &key mass mesh (color '(0.5 0.5 0.5 1.0)) types (scale 1.0)
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
    (make-item world name (or types (list mesh))
               (list
                (make-instance 'rigid-body
                  :name name :mass mass :pose (ensure-pose pose)
                  :collision-shape
                  (make-instance 'convex-hull-mesh-shape
                    :points (physics-utils:3d-model-vertices mesh-model)
                    :faces (physics-utils:3d-model-faces mesh-model)
                    :color color
                    :disable-face-culling disable-face-culling))))))

(defmethod add-object ((world bt-world) (type (eql :cutlery)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) cutlery-type)
  (let ((size (cutlery-dimensions cutlery-type)))
    (assert size)
    (make-item world name (list type cutlery-type)
               (list
                (make-instance 'rigid-body
                  :name name :mass mass :pose (ensure-pose pose)
                  :collision-shape (make-instance 'colored-box-shape
                                     :half-extents size
                                     :color color))))))

(defmethod add-object ((world bt-world) (type (eql :pancake-maker)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) size)
  (assert size)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-cylinder-shape
                                   :half-extents (ensure-vector size)
                                   :color color)))))

(defmethod add-object ((world bt-world) (type (eql :pancake)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) size)
  (assert size)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-cylinder-shape
                                   :half-extents (ensure-vector size)
                                   :color color)))))

(defmethod add-object ((world bt-world) (type (eql :orange)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) radius)
  (assert radius)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-sphere-shape
                                   :radius radius
                                   :color color)))))

(defmethod add-object ((world bt-world) (type (eql :apple)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) radius)
  (assert radius)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-sphere-shape
                                   :radius radius
                                   :color color)))))

(defmethod add-object ((world bt-world) (type (eql :sugar-box)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) size)
  (assert size)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-box-shape
                                   :half-extents (ensure-vector size)
                                   :color color)))))

(defmethod add-object ((world bt-world) (type (eql :cereal)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) size)
  (assert size)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-box-shape
                                   :half-extents (ensure-vector size)
                                   :color color)))))
