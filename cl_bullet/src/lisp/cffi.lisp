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

(define-foreign-library bullet-dynamics
  (:unix "libBulletDynamics.so"))

#.(loop for path in (ros-library-paths ros-load:*current-ros-package*)
        do (pushnew (concatenate 'string path "/")
                    *foreign-library-directories*
                    :test #'equal))

(use-foreign-library bullet-dynamics)

(defcfun ("plNewBulledSdk" new-bullet-sdk) :pointer)

(defcfun ("plDeletePhysicsSdk" delete-bullet-sdk) :void
  (sdk-handle :pointer))

(defcfun ("plCreateSapBroadphase" create-broadphase) :pointer
  (begin-callback :pointer)
  (end-callback :pointer))

(defcfun ("plDestroyBroadphase" destroy-broadphase) :void
  (broadphase-handle :pointer))

(defcfun ("plCreateProxy" create-broadphase-proxy) :pointer
  (broadphase-handle :pointer)
  (client-data :pointer)
  (min-x :double)
  (min-y :double)
  (min-z :double)
  (max-x :double)
  (max-y :double)
  (max-z :double))

(defcfun ("plDestroyProxy" destroy-broadphase-proxy) :void
  (broadphase-handle :pointer)
  (broadphase-proxy-handle :pointer))

(defcfun ("plSetBoundingBox" set-broadphase-proxy-bounding-box) :void
  (broadphase-proxy-handle :pointer)
  (min-x :double)
  (min-y :double)
  (min-z :double)
  (max-x :double)
  (max-y :double)
  (max-z :double))

(defcfun ("plCreateDynamicsWorld" create-dynamics-world) :pointer
  (sdk-handle :pointer))

(defcfun ("plDeleteDynamicsWorld" delete-dynamics-world) :void
  (dynamics-world-handle :pointer))

(defcfun ("plStepSimulation" step-simulation) :void
  (dynamics-world-handle :pointer)
  (time-step :double))

(defcfun ("plAddRigidBody" add-rigid-body) :void
  (dynamics-world-handle :pointer)
  (rigid-body-handle :pointer))

(defcfun ("plRemoveRigidBody" remove-rigid-body) :void
  (dynamics-world-handle :pointer)
  (rigid-body-handle :pointer))

(defcfun ("plCreateRigidBody" create-rigid-body) :pointer
  (user-data :pointer)
  (mass :float)
  (collision-shape-handle :pointer))

(defcfun ("plDeleteRigidBody" delete-rigid-body) :pointer
  (rigid-body-handle :pointer))

(defcfun ("plNewSphereShape" new-sphere-shape) :pointer
  (radius :double))

(defcfun ("plNewBoxShape" new-box-shape) :pointer
  (length :double)
  (width :double)
  (height :double))

(defcfun ("plNewCapsuleShape" new-capsule-shape) :pointer
  (radius :double)
  (height :double))

(defcfun ("plNewConeShape" new-cone-shape) :pointer
  (radius :double)
  (height :double))

(defcfun ("plNewCylinderShape" new-cylinder-shape) :pointer
  (radius :double)
  (height :double))

(defcfun ("plNewCompoundShape" new-compoind-shape) :pointer)

(defcfun ("plAddChildShape" add-child-shape) :void
  (compound-shape-handle :pointer)
  (child-shape-handle :pointer)
  (position bt-3d-vector)
  (rotation bt-quaternion))

(defcfun ("plDeleteShape" delete-shape) :void
  (shape-handle :pointer))

(defcfun ("plNewConvexHullShape" new-convex-hull-shape) :pointer)

(defcfun ("plAddVertex" add-vertex) :void
  (convex-hull-handle :pointer)
  (x :double)
  (y :double)
  (z :double))

(defcfun ("plNewMeshInterface" new-mesh-interface) :pointer)

(defcfun ("plAddTriangle" add-triangle) :void
  (mesh-interface-handle :pointer)
  (v-0 bt-3d-vector)
  (v-1 bt-3d-vector)
  (v-2 bt-3d-vector))

(defcfun ("plSetScaling" set-scaling) :pointer
  (collision-shape-handle :pointer)
  (scaling bt-3d-vector))

(defun get-position (rigid-body-handle)
  (with-foreign-object (result :double 3)
    (foreign-funcall "plGetPosition" :pointer rigid-body-handle :pointer result)
    (translate-from-foreign result (make-instance 'bt-3d-vector))))

(defun get-orientation (rigid-body-handle)
  (with-foreign-object (result :double 4)
    (foreign-funcall "plGetOrientation" :pointer rigid-body-handle :pointer result)
    (translate-from-foreign result (make-instance 'bt-quaternion))))

(defcfun ("plSetPosition" set-position) :void
  (rigid-body-handle :pointer)
  (position bt-3d-vector))

(defcfun ("plSetOrientation" set-orientation) :void
  (rigid-body-handle :pointer)
  (position bt-quaternion))
