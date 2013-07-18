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

(in-package :bullet)

(define-foreign-ros-library bullet-cl "libbullet_cl.so" "cl_bullet")

(use-foreign-library bullet-cl)

(defcvar (*bullet-world-scaling-factor* "bulletWorldScalingFactor") :double)

;;; dynamics_world.cpp
(defcfun ("newDiscreteDynamicsWorld" new-discrete-dynamics-world) :pointer
  (gravity-vector bt-3d-vector))

(defcfun ("deleteDiscreteDynamicsWorld" delete-discrete-dynamics-world) :void
  (handle :pointer))

(defun get-gravity (world-handle)
  (with-foreign-object (result :double 3)
    (foreign-funcall "getGravity"
                     :pointer world-handle
                     :pointer result)
    (translate-from-foreign result (make-instance 'bt-3d-vector))))

(defcfun ("setGravity" set-gravity) :void
  (world-handle :pointer)
  (gravity bt-3d-vector))

(defcfun ("stepSimulation" cffi-step-simulation) :void
  (world-handle :pointer)
  (time-step :double))

(defcfun ("addConstraint" cffi-add-constraint) :void
  (world-handle :pointer)
  (constraint :pointer)
  (disable-collisions :boolean))

(defcfun ("removeConstraint" cffi-remove-constraint) :void
  (world-handle :pointer)
  (constraint :pointer))

(defcfun ("getNumConstraints" cffi-get-num-constraints) :int
  (world-handle :pointer))

(defcfun ("getConstraint" cffi-get-constraint) :pointer
  (world-handle :pointer)
  (index :int))

(defcfun ("addRigidBody" cffi-add-rigid-body) :void
  (world-handle :pointer)
  (body :pointer))

(defcfun ("addRigidBodyWithMask" cffi-add-rigid-body-with-mask) :void
  (world-handle :pointer)
  (body :pointer)
  (group collision-filters)
  (mask collision-filters))

(defcfun ("removeRigidBody" cffi-remove-rigid-body) :void
  (world-handle :pointer)
  (body :pointer))

(defcfun ("getNumRigidBodies" cffi-get-num-rigid-bodies) :int
  (world-handle :pointer))

(defcfun ("getRigidBody" cffi-get-rigid-body) :pointer
  (world-handle :pointer)
  (index :int))

(defcfun ("setDebugDrawer" cffi-set-debug-drawer) :void
  (world-handle :pointer)
  (drawer :pointer))

(defcfun ("getDebugDrawer" cffi-get-debug-drawer) :pointer
  (world-handle :pointer))

(defcfun ("debugDrawWorld" cffi-debug-draw-world) :void
  (world-handle :pointer))

(defcfun ("serializeWorld" serialize-world) :void
  (world-handle :pointer)
  (serializer :pointer))

(defcfun ("performDiscreteCollisionDetection" cffi-perform-collision-detection) :void
  (world-handle :pointer))

(defcfun ("getNumManifolds" get-num-manifolds) :int
  (world-handle :pointer))

(defcfun ("getManifoldByIndex" get-manifold-by-index) :pointer
  (world-handle :pointer)
  (index :int))

(defcfun ("manifoldGetBody0" manifold-get-body-0) :pointer
  (manifold :pointer))

(defcfun ("manifoldGetBody1" manifold-get-body-1) :pointer
  (manifold :pointer))

(defcfun ("manifoldGetNumContactPoints" manifold-get-num-contact-points) :int
  (manifold :pointer))

(defun manifold-get-contact-point-0 (manifold index)
  (with-foreign-object (result :double 3)
    (foreign-funcall "manifoldGetContactPoint0"
                     :pointer manifold :int index
                     :pointer result)
    (translate-from-foreign result (make-instance 'bt-3d-vector))))

(defun manifold-get-contact-point-1 (manifold index)
  (with-foreign-object (result :double 3)
    (foreign-funcall "manifoldGetContactPoint1"
                     :pointer manifold :int index
                     :pointer result)
    (translate-from-foreign result (make-instance 'bt-3d-vector))))

;;; rigid_body.cpp

(defcfun ("newRigidBody" new-rigid-body) :pointer
  (mass :double)
  (motion-state :pointer)
  (collision-shape :pointer))

(defcfun ("deleteRigidBody" delete-rigid-body) :void
  (body-handle :pointer))

(defun cffi-get-total-force (body-handle)
  (with-foreign-object (result :double 3)
    (foreign-funcall "getTotalForce" :pointer body-handle :pointer result)
    (translate-from-foreign result (make-instance 'bt-3d-vector))))

(defun cffi-get-total-torque (body-handle)
  (with-foreign-object (result :double 3)
    (foreign-funcall "getTotalTorque" :pointer body-handle :pointer result)
    (translate-from-foreign result (make-instance 'bt-3d-vector))))

(defcfun ("applyForce" cffi-apply-force) :void
  (body-handle :pointer)
  (force bt-3d-vector)
  (rel-pos bt-3d-vector))

(defcfun ("applyCentralForce" cffi-apply-central-force) :void
  (body-handle :pointer)
  (force bt-3d-vector))

(defcfun ("applyTorque" cffi-apply-torque) :void
  (body-handle :pointer)
  (torque bt-3d-vector))

(defun get-linear-velocity (body-handle)
  (with-foreign-object (result :double 3)
    (foreign-funcall "getLinearVelocity"
                     :pointer body-handle
                     :pointer result)
    (translate-from-foreign result (make-instance 'bt-3d-vector))))

(defcfun ("setLinearVelocity" set-linear-velocity) :void
  (body-handle :pointer)
  (velocity bt-3d-vector))

(defun get-angular-velocity (body-handle)
  (with-foreign-object (result :double 3)
    (foreign-funcall "getAngularVelocity"
                     :pointer body-handle
                     :pointer result)
    (translate-from-foreign result (make-instance 'bt-3d-vector))))

(defcfun ("setAngularVelocity" set-angular-velocity) :void
  (body-handle :pointer)
  (velocity bt-3d-vector))

(defcfun ("clearForces" cffi-clear-forces) :void
  (body-handle :pointer))

(defcfun ("getMotionState" get-motion-state) :pointer
  (body-handle :pointer))

(defcfun ("setMotionState" set-motion-state) :void
  (body-handle :pointer)
  (motion-state :pointer))

(defcfun ("setActivationState" set-activation-state) :void
  (body-handle :pointer)
  (new-state activation-state))

(defcfun ("getActivationState" get-activation-state) activation-state
  (body-handle :pointer))

(defcfun ("setCollisionFlags" set-collision-flags) :void
  (body-handle :pointer)
  (flags collision-flags))

(defcfun ("getCollisionFlags" get-collision-flags) collision-flags
  (body-handle :pointer))

(defcfun ("getCollisionShape" get-collision-shape) :pointer
  (body-handle :pointer))

(defun get-aabb (body-handle)
  (with-foreign-objects ((min :double 3)
                         (max :double 3))
    (foreign-funcall "getAabb"
                     :pointer body-handle
                     :pointer min :pointer max)
    (list
     (translate-from-foreign min (make-instance 'bt-3d-vector))
     (translate-from-foreign max (make-instance 'bt-3d-vector)))))

(defun get-collision-filter (body-handle)
  (with-foreign-objects ((group :int)
                         (mask :int))
    (foreign-funcall
     "getCollisionFilter" :pointer body-handle :pointer group :pointer mask)
    (values (mem-ref group :int)
            (mem-ref mask :int))))

(defcfun ("setCollisionFilter" set-collision-filter) :void
  (body-handle :pointer)
  (group collision-filters)
  (mask collision-filters))

(defcfun ("setMassProps" set-mass-props) :void
  (body-handle :pointer)
  (mass :double))

;;; motion_state.cpp

(defcfun ("newMotionState" new-motion-state) :pointer
  (transform bt-transform))

(defcfun ("deleteMotionState" delete-motion-state) :void
  (motion-state-handle :pointer))

(defcfun ("setCenterOfMass" set-center-of-mass) :void
  (motion-state-handle :pointer)
  (center-off-mass bt-3d-vector))

(defcfun ("setWorldTransform" set-world-transform) :void
  (motion-state-handle :pointer)
  (transform bt-transform))

(defun get-world-transform (motion-state-handle)
  (with-foreign-object (transform :double 7)
    (foreign-funcall "getWorldTransform"
                     :pointer motion-state-handle
                     :pointer transform)
    (translate-from-foreign transform (make-instance 'bt-transform))))

;;; collision_shapes.cpp

(defcfun ("deleteCollisionShape" delete-collision-shape) :void
  (shape-handle :pointer))

(defcfun ("getShapeType" get-shape-type) broadphase-native-type
  (shape-handle :pointer))

(defcfun ("newBoxShape" new-box-shape) :pointer
  (half-extents bt-3d-vector))

(defcfun ("isBoxShape" box-shape-p) :boolean
  (shape :pointer))

(defun get-box-half-extents (shape)
  (with-foreign-object (vec :double 3)
    (foreign-funcall "getBoxHalfExtents"
                     :pointer shape
                     :pointer vec)
    (translate-from-foreign vec (make-instance 'bt-3d-vector))))

(defcfun ("newStaticPlaneShape" new-static-plane-shape) :pointer
  (normal bt-3d-vector)
  (constant :double))

(defcfun ("isStaticPlaneShape" static-plane-shape-p) :boolean
  (shape :pointer))

(defcfun ("getPlaneNormal" get-plane-normal) :double
  (shape :pointer))

(defcfun ("getPlaneConstant" get-plane-constant) :double
  (shape :pointer))

(defcfun ("newSphereShape" new-sphere-shape) :pointer
  (radius :double))

(defcfun ("isSphereShape" sphere-shape-p) :boolean
  (shape :pointer))

(defcfun ("getSphereRadius" get-sphere-radius) :double
  (shape :pointer))

(defcfun ("newCylinderShape" new-cylinder-shape) :pointer
  (half-extents bt-3d-vector))

(defcfun ("isCylinderShape" cylinder-shape-p) :boolean
  (shape :pointer))

(defun get-cylinder-half-extents (shape)
  (with-foreign-object (vec :double 3)
    (foreign-funcall "getCylinderHalfExtents"
                     :pointer shape
                     :pointer vec)
    (translate-from-foreign vec (make-instance 'bt-3d-vector))))

(defcfun ("newConeShape" new-cone-shape) :pointer
  (radius :double)
  (height :double))

(defcfun ("isConeShape" cone-shape-p) :boolean
  (shape :pointer))

(defcfun ("getConeRadius" get-cone-radius) :double
  (shape :pointer))

(defcfun ("getConeHeight" get-cone-height) :double
  (shape :pointer))

(defcfun ("newCompoundShape" new-compound-shape) :pointer)

(defcfun ("isCompoundShape" compound-shape-p) :boolean
  (shape :pointer))

(defcfun ("addChildShape" cffi-add-child-shape) :void
  (parent :pointer)
  (position bt-3d-vector)
  (orientation bt-quaternion)
  (shape :pointer))

(defcfun ("getNumChildShapes" get-num-child-shapes) :int
  (shape :pointer))

(defcfun ("getChildShape" cffi-get-child-shape) :pointer
  (shape :pointer)
  (index :int))

(defun cffi-get-child-transform (shape)
  (with-foreign-object (vec :double 7)
    (foreign-funcall "getChildTransform"
                     :pointer shape
                     :pointer vec)
    (translate-from-foreign vec (make-instance 'bt-transform))))

(defun new-convex-hull-shape (points)
  (flet ((points->foreign (native-vector points)
           (let ((points (etypecase points
                           (list
                            (make-array (length points)
                                        :initial-contents points))
                           (array
                            points))))
             (loop for i below (* (length points) 3) by 3
                   for p across points
                   do (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                          p
                        (setf (mem-aref native-vector :double i)
                              (coerce (cl-transforms:x p) 'double-float))
                        (setf (mem-aref native-vector :double (+ i 1))
                              (coerce (cl-transforms:y p) 'double-float))
                        (setf (mem-aref native-vector :double (+ i 2))
                              (coerce (cl-transforms:z p) 'double-float)))))
           points))
    (with-foreign-object (native-points :double (* 3 (length points)))
      (points->foreign native-points points)
      (foreign-funcall "newConvexHullShape" :pointer native-points :int (length points) :pointer))))

(defcfun ("isConvexHullShape" convex-hull-shape-p) :boolean
  (shape :pointer))

(defcfun ("addPoint" cffi-add-point) :void
  (shape :pointer)
  (point bt-3d-vector))

(defcfun ("convexHullGetNumPoints" convex-hull-get-num-points) :int
  (shape :pointer))

(defun cffi-get-point (shape index)
  (with-foreign-object (vec :double 3)
    (foreign-funcall "getPoint"
                     :pointer shape
                     :int index
                     :pointer vec)
    (translate-from-foreign vec (make-instance 'bt-3d-vector))))

;;; constraints.cpp

(defcfun ("deleteConstraint" delete-constraint) :void
  (constant :pointer))

;;; point-2-point constraint
(defcfun ("newPoint2PointConstraint" new-point-2-point-constraint) :pointer
  (rb-a :pointer)
  (rb-b :pointer)
  (pivot-in-a bt-3d-vector)
  (pivot-in-b bt-3d-vector))

(defcfun ("isPoint2PointConstraint" point-2-point-constraint-p) :boolean
  (constraint :pointer))

;;; hinge constraint
(defcfun ("newHingeConstraint" new-hinge-constraint) :pointer
  (rb-a :pointer)
  (rb-b :pointer)
  (frame-in-a bt-transform)
  (frame-in-b bt-transform))

(defcfun ("isHingeConstraint" hinge-constraint-p) :boolean
  (constraint :pointer))

(defcfun ("setAngularOnly" set-angular-only) :void
  (hinge :pointer)
  (angular-only :boolean))

(defcfun ("getAngularOnly" get-angular-only) :boolean
  (hinge :pointer))

(defcfun ("enableAngularMotor" enable-angular-motor) :void
  (hinge :pointer)
  (enable-motor :boolean)
  (target-velocity :double)
  (max-motor-impulse :double))

(defcfun ("enableMotor" enable-motor) :void
  (hinge :pointer)
  (enable-motor :boolean))

(defcfun ("getEnableMotor" get-enable-motor) :boolean
  (hinge :pointer))

(defcfun ("setMaxMotorImpulse" set-max-motor-impulse) :void
  (hinge :pointer)
  (max-motor-impulse :double))

(defcfun ("getMaxMotorImpulse" get-max-motor-impulse) :double
  (hinge :pointer))

(defcfun ("getMotorTargetVelocity" get-motor-target-velocity) :double
  (hinge :pointer))

(defcfun ("setMotorTarget" set-motor-target) :void
  (hinge :pointer)
  (target-angle :double)
  (dt :double))

(defcfun ("setLimit" set-limit) :void
  (hinge :pointer)
  (low :double)
  (high :double))

(defcfun ("setLimitComplex" set-limit-complex) :void
  (hinge :pointer)
  (low :double)
  (high :double)
  (softness :double)
  (bias-factor :double)
  (relaxation-factor :double))

(defcfun ("getHingeAngle" get-hinge-angle) :double
  (hinge :pointer))

(defcfun ("getLowerLimit" get-lower-limit) :double
  (hinge :pointer))

(defcfun ("getUpperLimit" get-upper-limit) :double
  (hinge :pointer))

;;; slider constraint

(defcfun ("newSliderConstraint" new-slider-constraint) :pointer
  (rb-a :pointer)
  (rb-b :pointer)
  (frame-in-a bt-transform)
  (frame-in-b bt-transform))

(defcfun ("isSliderConstraint" slider-constraint-p) :boolean
  (constraint :pointer))

(defcfun ("getLowerLinLimit" get-lower-lin-limit) :double
  (slider :pointer))

(defcfun ("setLowerLinLimit" set-lower-lin-limit) :void
  (slider :pointer)
  (lower-limit :double))

(defcfun ("getUpperLinLimit" get-upper-lin-limit) :double
  (slider :pointer))

(defcfun ("setUpperLinLimit" set-upper-lin-limit) :void
  (slider :pointer)
  (upper-limit :double))

(defcfun ("getLowerAngLimit" get-lower-ang-limit) :double
  (slider :pointer))

(defcfun ("setLowerAngLimit" set-lower-ang-limit) :void
  (slider :pointer)
  (lower-limit :double))

(defcfun ("getUpperAngLimit" get-upper-ang-limit) :double
  (slider :pointer))

(defcfun ("setUpperAngLimit" set-upper-ang-limit) :void
  (slider :pointer)
  (upper-limit :double))

(defcfun ("getSoftnessDirLin" get-softness-dir-lin) :double
  (slider :pointer))

(defcfun ("getRestitutionDirLin" get-restitution-dir-lin) :double
  (slider :pointer))

(defcfun ("getDampingDirLin" get-damping-dir-lin) :double
  (slider :pointer))

(defcfun ("getSoftnessDirAng" get-softness-dir-ang) :double
  (slider :pointer))

(defcfun ("getRestitutionDirAng" get-restitution-dir-ang) :double
  (slider :pointer))

(defcfun ("getDampingDirAng" get-damping-dir-ang) :double
  (slider :pointer))

(defcfun ("getSoftnessLimLin" get-softness-lim-lin) :double
  (slider :pointer))

(defcfun ("getRestitutionLimLin" get-restitution-lim-lin) :double
  (slider :pointer))

(defcfun ("getDampingLimLin" get-damping-lim-lin) :double
  (slider :pointer))

(defcfun ("getSoftnessLimAng" get-softness-lim-ang) :double
  (slider :pointer))

(defcfun ("getRestitutionLimAng" get-restitution-lim-ang) :double
  (slider :pointer))

(defcfun ("getDampingLimAng" get-damping-lim-ang) :double
  (slider :pointer))

(defcfun ("getSoftnessOrthoLin" get-softness-ortho-lin) :double
  (slider :pointer))

(defcfun ("getRestitutionOrthoLin" get-restitution-ortho-lin) :double
  (slider :pointer))

(defcfun ("getDampingOrthoLin" get-damping-ortho-lin) :double
  (slider :pointer))

(defcfun ("getSoftnessOrthoAng" get-softness-ortho-ang) :double
  (slider :pointer))

(defcfun ("getRestitutionOrthoAng" get-restitution-ortho-ang) :double
  (slider :pointer))

(defcfun ("getDampingOrthoAng" get-damping-ortho-ang) :double
  (slider :pointer))

(defcfun ("setSoftnessDirLin" set-softness-dir-lin) :void
  (slider :pointer)
  (value :double))

(defcfun ("setRestitutionDirLin" set-restitution-dir-lin) :void
  (slider :pointer)
  (value :double))

(defcfun ("setDampingDirLin" set-damping-dir-lin) :void
  (slider :pointer)
  (value :double))

(defcfun ("setSoftnessDirAng" set-softness-dir-ang) :void
  (slider :pointer)
  (value :double))

(defcfun ("setRestitutionDirAng" set-restitution-dir-ang) :void
  (slider :pointer)
  (value :double))

(defcfun ("setDampingDirAng" set-damping-dir-ang) :void
  (slider :pointer)
  (value :double))

(defcfun ("setSoftnessLimLin" set-softness-lim-lin) :void
  (slider :pointer)
  (value :double))

(defcfun ("setRestitutionLimLin" set-restitution-lim-lin) :void
  (slider :pointer)
  (value :double))

(defcfun ("setDampingLimLin" set-damping-lim-lin) :void
  (slider :pointer)
  (value :double))

(defcfun ("setSoftnessLimAng" set-softness-lim-ang) :void
  (slider :pointer)
  (value :double))

(defcfun ("setRestitutionLimAng" set-restitution-lim-ang) :void
  (slider :pointer)
  (value :double))

(defcfun ("setDampingLimAng" set-damping-lim-ang) :void
  (slider :pointer)
  (value :double))

(defcfun ("setSoftnessOrthoLin" set-softness-ortho-lin) :void
  (slider :pointer)
  (value :double))

(defcfun ("setRestitutionOrthoLin" set-restitution-ortho-lin) :void
  (slider :pointer)
  (value :double))

(defcfun ("setDampingOrthoLin" set-damping-ortho-lin) :void
  (slider :pointer)
  (value :double))

(defcfun ("setSoftnessOrthoAng" set-softness-ortho-ang) :void
  (slider :pointer)
  (value :double))

(defcfun ("setRestitutionOrthoAng" set-restitution-ortho-ang) :void
  (slider :pointer)
  (value :double))

(defcfun ("setDampingOrthoAng" set-damping-ortho-ang) :void
  (slider :pointer)
  (value :double))

(defcfun ("setPoweredLinMotor" set-powered-lin-motor) :void
  (slider :pointer)
  (on-off :boolean))

(defcfun ("getPoweredLinMotor" get-powered-lin-motor) :boolean
  (slider :pointer))

(defcfun ("setTargetLinMotorVelocity" set-target-lin-motor-velocity) :void
  (slider :pointer)
  (target-lin-motor-velocity :double))

(defcfun ("getTargetLinMotorVelocity" get-target-lin-motor-velocity) :double
  (slider :pointer))

(defcfun ("setMaxLinMotorForce" set-max-lin-motor-force) :void
  (slider :pointer)
  (max-lin-motor-force :double))

(defcfun ("getMaxLinMotorForce" get-max-lin-motor-force) :double
  (slider :pointer))

(defcfun ("setPoweredAngMotor" set-powered-ang-motor) :void
  (slider :pointer)
  (on-off :boolean))

(defcfun ("getPoweredAngMotor" get-powered-ang-motor) :boolean
  (slider :pointer))

(defcfun ("setTargetAngMotorVelocity" set-target-ang-motor-velocity) :void
  (slider :pointer)
  (target-ang-motor-velocity :double))

(defcfun ("getTargetAngMotorVelocity" get-target-ang-motor-velocity) :double
  (slider :pointer))

(defcfun ("setMaxAngMotorForce" set-max-ang-motor-force) :void
  (slider :pointer)
  (max-ang-motor-force :double))

(defcfun ("getMaxAngMotorForce" get-max-ang-motor-force) :double
  (slider :pointer))

(defcfun ("getLinearPos" get-linear-pos) :double
  (slider :pointer))

;;; debug_draw.cpp

(defcfun ("newCLBulletDebugDraw" new-cl-bullet-debug-draw) :pointer
  (callbacks :pointer)
  (arg :pointer))

(defcfun ("deleteCLBulletDebugDraw" delete-cl-bullet-debug-draw) :void
  (handle :pointer))

(defcfun ("setDebugMode" set-debug-mode) :void
  (draw :pointer)
  (mode debug-modes))

(defcfun ("getDebugMode" get-debug-mode) debug-modes
  (draw :pointer))

(defcfun ("setCallbacks" set-callbacks) :void
  (draw :pointer)
  (callbacks :pointer))

(defcfun ("getCallbacks" get-callbacks) :pointer
  (draw :pointer))
