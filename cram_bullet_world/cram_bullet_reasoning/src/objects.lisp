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

(defgeneric add-object (world type name pose &key &allow-other-keys)
  (:documentation "Adds an object of `type' named `name' to the bullet
  world `world'. The object is placed at `pose'"))

(defgeneric remove-object (world name)
  (:documentation "Removes the object named `name' from the `world'")
  (:method ((world bt-reasoning-world) name)
    (let ((obj (object world name)))
      (cond (obj
             (mapcar (lambda (body) (remove-rigid-body world body))
                     (rigid-bodies obj))
             (remhash name (slot-value world 'objects))
             (setf (slot-value obj 'world) nil))
            (t (warn 'simple-warning
                     :format-control "Could not find a body named `~a'"
                     :format-arguments (list name)))))))

(defclass object ()
  ((name :initarg :name :reader name)
   (rigid-bodies :initform (make-hash-table :test 'equal))
   (pose-reference-body :initarg :pose-reference-body
                        :documentation "The name of the rigid-body
                        that is used for returning the pose of the
                        object. It defaults to the first body that got
                        added.")
   (world :initarg :world :reader world)))

(defgeneric rigid-bodies (obj)
  (:documentation "Return the list of rigid bodies that belong to the object `obj'")
  (:method ((obj object))
    (loop for name being the hash-keys in (slot-value obj 'rigid-bodies)
          using (hash-value body)
          collecting (or body (rigid-body obj name)))))

(defgeneric rigid-body-names (obj)
  (:documentation "Return the list of rigid bodies that belong to the object `obj'")
  (:method ((obj object))
    (loop for body being the hash-keys in (slot-value obj 'rigid-bodies)
          collecting body)))

(defgeneric rigid-body (obj name)
  (:documentation "Returns the rigid body named `name' or NIL if the body doesn't exist")
  (:method ((obj object) name)
    (with-slots (world rigid-bodies) obj
      (multiple-value-bind (value valid)
          (gethash name rigid-bodies)
        (when valid
          (or value
              (setf (gethash name rigid-bodies)
                    (or
                     (find name (bodies world)
                           :key #'name
                           :test #'equal)
                     (error 'simple-error
                            :format-control "Could not find body with name `~a'"
                            :format-arguments (list name))))))))))

(defgeneric copy-object (obj world)
  (:documentation "Copies the object `obj' and makes it an object of world `world'")
  (:method ((obj object) (world bt-reasoning-world))
    (with-slots (name rigid-bodies pose-reference-body) obj
      (let ((new-instance
             (make-instance
              'object :name name
              :pose-reference-body pose-reference-body
              :world world)))
        (prog1 new-instance
          (setf (slot-value new-instance 'rigid-bodies)
                (copy-hash-table rigid-bodies)))))))

(defgeneric initialize-rigid-bodies (object rigid-bodies &key add)
  (:method ((object object) rigid-bodies &key (add t))
    (with-slots (world) object
      (declare (type list rigid-bodies))
      (assert (eql (hash-table-count (slot-value object 'rigid-bodies)) 0))
      (assert rigid-bodies)
      (dolist (body rigid-bodies)
        (when (and add world)
          (when (typep world 'bt-reasoning-world)
            (setf (gethash (name object) (slot-value world 'objects))
                  object))
          (add-rigid-body world body))
        (setf (gethash (name body) (slot-value object 'rigid-bodies))
              body)))))

(defun make-object (world name &optional
                    bodies (add-to-world t))
  (make-instance 'object
                 :name name
                 :world world
                 :rigid-bodies bodies
                 :add add-to-world))

(defmethod initialize-instance :after ((object object)
                                       &key rigid-bodies pose-reference-body
                                         (add t))
  (when rigid-bodies
    (initialize-rigid-bodies object rigid-bodies :add add)
    (unless pose-reference-body
      (setf (slot-value object 'pose-reference-body) (name (car rigid-bodies))))))

(defmethod invalidate-object :after ((obj object))
  (with-slots (rigid-bodies) obj
    (loop for key being the hash-keys in rigid-bodies do
      (setf (gethash key rigid-bodies) nil))))

(defmethod pose ((object object))
  "Returns the pose of the object, i.e. the pose of the body named by
  the slot `pose-reference-body'"
  (let ((body (rigid-body object (slot-value object 'pose-reference-body))))
    (when body
      (pose body))))

(defmethod (setf pose) (new-value (object object))
  (let ((body (rigid-body object (slot-value object 'pose-reference-body))))
    (when body
      (setf (pose body) new-value))))

(defun set-object-pose (object new-pose)
  (setf (pose object) (ensure-pose new-pose)))

(defmethod draw ((context gl-context) (object object))
  (dolist (body (rigid-bodies object))
    (draw context body)))

(defun make-rigid-body-name (obj-name body-name)
  (flet ((ensure-string (name)
           (etypecase name
             (string name)
             (symbol (symbol-name name)))))
    (intern (concatenate
             'string
             (ensure-string obj-name)
             "."
             (ensure-string body-name))
            :keyword)))

(defmethod add-object ((world bt-reasoning-world) type name pose
                       &key disable-collisions-with)
  (prog1
      (call-next-method)
    (when disable-collisions-with
      (disable-collisions world name disable-collisions-with))))

(defmethod add-object ((world bt-world) (type (eql :box)) name pose &key mass size)
  (destructuring-bind (size-x size-y size-z) size
    (make-object world name
                 (list
                  (make-instance
                   'rigid-body
                   :name name :mass mass :pose (ensure-pose pose)
                   :collision-shape (make-instance
                                     'box-shape
                                     :half-extents (cl-transforms:make-3d-vector
                                                    (/ size-x 2)
                                                    (/ size-y 2)
                                                    (/ size-z 2))))))))

(defmethod add-object ((world bt-world) (type (eql :colored-box)) name pose
                       &key mass size color)
  (destructuring-bind (size-x size-y size-z) size
    (make-object world name
                 (list
                  (make-instance
                      'rigid-body
                    :name name :mass mass :pose (ensure-pose pose)
                    :collision-shape (make-instance
                                         'colored-box-shape
                                       :half-extents (cl-transforms:make-3d-vector
                                                      (/ size-x 2)
                                                      (/ size-y 2)
                                                      (/ size-z 2))
                                       :color color))))))

(defmethod add-object ((world bt-world) (type (eql :static-plane)) name pose
                       &key normal constant)
  (destructuring-bind (normal-x normal-y normal-z) normal
    (make-object world name
                 (list
                  (make-instance
                   'rigid-body
                   :name name :pose (ensure-pose pose)
                   :group :static-filter
                   :collision-shape (make-instance
                                     'textured-static-plane-shape
                                     :normal (cl-transforms:make-3d-vector
                                              normal-x normal-y normal-z)
                                     :constant constant
                                     :width 16 :height 16
                                     :texture (texture-str->bitmap
                                               *static-plane-texture*
                                               #\Space)))))))

(defmethod add-object ((world bt-world) (type (eql :sphere)) name pose
                       &key mass radius color)
  (make-object world name
               (list
                (make-instance
                 'rigid-body
                 :name name :mass mass :pose (ensure-pose pose)
                  :collision-shape (make-instance 'colored-sphere-shape
                                     :radius radius :color color)))))

(defmethod add-object ((world bt-world) (type (eql :cylinder)) name pose &key mass size)
  (destructuring-bind (size-x size-y size-z) size
    (make-object world name
                 (list
                  (make-instance
                   'rigid-body
                   :name name :mass mass :pose (ensure-pose pose)
                   :collision-shape (make-instance 'cylinder-shape
                                                   :half-extents (cl-transforms:make-3d-vector
                                                                  (/ size-x 2)
                                                                  (/ size-y 2)
                                                                  (/ size-z 2))))))))

(defmethod add-object ((world bt-world) (type (eql :cone)) name pose &key
                       mass radius height)
  (make-object world name
               (list
                (make-instance
                 'rigid-body
                 :name name :mass mass :pose (ensure-pose pose)
                 :collision-shape (make-instance 'cone-shape
                                                 :radius radius
                                                 :height height)))))

(defmethod add-object ((world bt-world) (type (eql :point-cloud)) name pose &key points)
  (make-object world name
               (list
                (make-instance
                 'rigid-body
                 :name name :mass 0.0 :pose (ensure-pose pose)
                 :collision-shape (make-instance 'convex-hull-shape :points points)))))
