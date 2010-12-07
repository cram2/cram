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

(in-package :bt)

;;;
;;; Currently, the following functions from Bullet-C-Api.h are missing:
;;; 
;;; * plCreateCollisionWorld
;;; 
;;; * plGetOpenGLMatrix
;;; 
;;; * plSetEuler
;;; 
;;; * plSetOpenGLMatrix
;;; 
;;; * plRayCast (this function cannot be implemented so easily since
;;;   it requires a struct to be passed by value. We will need to wrap
;;;   it.)
;;; 
;;; * plNearestPoints
;;; 

(define-foreign-library bullet-cl
  (:unix "libbullet_cl.so"))

#.(loop for path in (ros-library-paths ros-load:*current-ros-package*)
        do (pushnew (concatenate 'string path "/")
                    *foreign-library-directories*
                    :test #'equal))

(use-foreign-library bullet-cl)


;;; dynamics_world.cpp
(defcfun ("newDiscreteDynamicsWorld" new-discrete-dynamics-world) :pointer
  (gravity-vector bt-3d-vector))

(defcfun ("deleteDiscreteDynamicsWorld" delete-discrete-dynamics-world) :void
  (handle :pointer))

(defcfun ("stepSimulation" step-simulation) :void
  (world-handle :pointer)
  (time-step :double))

(defcfun ("addConstraint" add-constraint) :void
  (world-handle :pointer)
  (constraint :pointer)
  (disable-collisions :boolean))

(defcfun ("removeConstraint" remove-constraint) :void
  (world-handle :pointer)
  (constraint :pointer))

(defcfun ("addRigidBody" add-rigid-body) :void
  (world-handle :pointer)
  (body :pointer))

(defcfun ("removeRigidBody" remove-rigid-body) :void
  (world-handle :pointer)
  (body :pointer))

;;; rigid_body.cpp

(defcfun ("newRigidBody" new-rigid-body) :pointer
  (mass :double)
  (motion-state :pointer)
  (collision-shape :pointer))

(defcfun ("deleteRigidBody" delete-rigid-body) :void
  (body-handle :pointer))

(defun get-total-force (body-handle)
  (with-foreign-object (result :double 3)
    (foreign-funcall "getTotalForce" :pointer body-handle :pointer result)
    (translate-from-foreign result (make-instance 'bt-3d-vector))))

(defcfun ("getMotionState" get-motion-state) :pointer
  (body-handle :pointer))

(defcfun ("setMotionState" set-motion-state) :void
  (body-handle :pointer)
  (motion-state :pointer))

;;; motion_state.cpp

(defcfun ("newMotionState" new-motion-state) :pointer
  (position bt-3d-vector)
  (orientation bt-quaternion))

(defcfun ("deleteMotionState" delete-motion-state) :void
  (motion-state-handle :pointer))

(defcfun ("setCenterOfMass" set-center-of-mass) :void
  (motion-state-handle :pointer)
  (center-off-mass bt-3d-vector))

(defun get-world-transform (motion-state-handle)
  (with-foreign-objects ((position :double 3)
                         (orientation :double 4))
    (foreign-funcall "getWorldTransform"
                     :pointer motion-state-handle
                     :pointer position :pointer orientation)
    (cl-transforms:make-transform
     (translate-from-foreign position (make-instance 'bt-3d-vector))
     (translate-from-foreign orientation (make-instance 'bt-quaternion)))))

;;; collision_shapes.cpp

(defcfun ("deleteCollisionShape" delete-collision-shape) :void
  (shape-handle :pointer))

(defcfun ("newBoxShape" new-box-shape) :pointer
  (half-extents bt-3d-vector))

(defcfun ("isBoxShape" box-shape-p) :boolean
  (shape :pointer))

(defcfun ("newStaticPlaneShape" new-static-plane-shape) :pointer
  (normal bt-3d-vector)
  (constant :double))

(defcfun ("isStaticPlaneShape" static-plane-shape-p) :boolean
  (shape :pointer))

(defcfun ("newSphereShape" new-sphere-shape) :pointer
  (radius :double))

(defcfun ("isSphereShape" sphere-shape-p) :boolean
  (shape :pointer))

(defcfun ("newCylinderShape" new-cyliner-shape) :pointer
  (half-extents bt-3d-vector))

(defcfun ("isCylinerShape" cylinder-shape-p) :boolean
  (shape :pointer))

(defcfun ("newConeShape" new-cone-shape) :pointer
  (radius :double)
  (height :double))

(defcfun ("isConeShape" cone-shape-p) :boolean
  (shape :pointer))

(defcfun ("newCompoundShape" new-compound-shape) :pointer)

(defcfun ("isCompoundShape" compound-shape-p) :boolean
  (shape :pointer))

(defcfun ("addChildShape" add-child-shape) :void
  (parent :pointer)
  (position bt-3d-vector)
  (orientation bt-quaternion)
  (shape :pointer))

(defun new-convex-hull-shape (points)
  (flet ((points->foreign (native-vector points)
           (loop for i below (* (length points) 3) by 3
                 for p in points
                 do (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                        p
                      (setf (mem-aref native-vector i) (coerce cl-transforms:x 'double-float))
                      (setf (mem-aref native-vector (+ i 1)) (coerce cl-transforms:y 'double-float))
                      (setf (mem-aref native-vector (+ i 2)) (coerce cl-transforms:z 'double-float))))
           points))
    (with-foreign-object (native-points :double (* 3 (length points)))
      (points->foreign native-points points)
      (foreign-funcall "newConvexHullShape" :pointer native-points :int (length points)))))

(defcfun ("isConvexHullShape" convex-hull-shape-p) :boolean
  (shape :pointer))

(defcfun ("addPoint" add-point) :void
  (shape :pointer)
  (point bt-3d-vector))
