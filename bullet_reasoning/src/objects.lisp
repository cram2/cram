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
  (:method ((world bt-world) name)
    (let ((body (find name (bodies world) :key #'name :test #'equal)))
      (if body
          (remove-rigid-body world body)
          (warn 'simple-warning :format-control "Could not find a body named `~a'" name)))))

(defun ensure-pose (pose)
  (etypecase pose
    (list (destructuring-bind
                ((x y z) (ax ay az aw))
              pose
            (cl-transforms:make-pose
             (cl-transforms:make-3d-vector x y z)
             (cl-transforms:make-quaternion ax ay az aw))))
    (cl-transforms:pose pose)))

(defmethod add-object ((world bt-world) (type (eql 'box)) name pose &key mass size)
  (let ((pose (ensure-pose pose)))
    (destructuring-bind (size-x size-y size-z) size
      (add-rigid-body world (make-instance
                             'rigid-body
                             :name name :mass mass :pose pose
                             :collision-shape (make-instance
                                               'box-shape
                                               :half-extents (cl-transforms:make-3d-vector
                                                              (/ size-x 2)
                                                              (/ size-y 2)
                                                              (/ size-z 2))))))))

(defmethod add-object ((world bt-world) (type (eql 'static-plane)) name pose &key
                       normal constant)
  (let ((pose (ensure-pose pose)))
    (destructuring-bind (normal-x normal-y normal-z) normal
      (add-rigid-body world (make-instance
                             'rigid-body
                             :name name :pose pose
                             :collision-shape (make-instance
                                               'static-plane-shape
                                               :normal (cl-transforms:make-3d-vector
                                                        normal-x normal-y normal-z)
                                               :constant constant))))))

(defmethod add-object ((world bt-world) (type (eql 'sphere)) name pose &key mass radius)
  (let ((pose (ensure-pose pose)))
    (add-rigid-body world (make-instance
                          'rigid-body
                          :name name :mass mass :pose pose
                          :collision-shape (make-instance 'sphere-shape :radius radius)))))

(defmethod add-object ((world bt-world) (type (eql 'cylinder)) name pose &key mass size)
  (let ((pose (ensure-pose pose)))
    (destructuring-bind (size-x size-y size-z) size
      (add-rigid-body world (make-instance
                             'rigid-body
                             :name name :mass mass :pose pose
                             :collision-shape (make-instance 'cylinder-shape
                                                             :half-extents (cl-transforms:make-3d-vector
                                                                            (/ size-x 2)
                                                                            (/ size-y 2)
                                                                            (/ size-z 2))))))))

(defmethod add-object ((world bt-world) (type (eql 'cone)) name pose &key
                       mass radius height)
  (let ((pose (ensure-pose pose)))
    (add-rigid-body world (make-instance
                           'rigid-body
                           :name name :mass mass :pose pose
                           :collision-shape (make-instance 'cone-shape
                                                           :radius radius
                                                           :height height)))))
