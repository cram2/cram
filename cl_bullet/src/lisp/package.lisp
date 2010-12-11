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

(defpackage cl-bullet
    (:nicknames :bullet :bt)
  (:use #:common-lisp #:cffi)
  (:export
   ;; dynamics world
   new-discrete-dynamics-world
   delete-discrete-dynamics-world
   step-simulation
   add-constraint
   remove-constraint
   add-rigid-body
   remove-rigid-body
   set-debug-drawer
   get-debug-drawer
   debug-draw-world
   ;; rigid bodies
   new-rigid-body
   delete-rigid-body
   get-total-force
   get-motion-state
   set-motion-state
   set-activation-state
   set-collision-flags
   ;; motion states
   new-motion-state
   delete-motion-state
   set-center-of-mass
   get-world-transform
   ;; collision shapes
   delete-collision-shape
   new-box-shape
   box-shape-p
   new-static-plane-shape
   static-plane-shape-p
   new-sphere-shape
   sphere-shape-p
   new-cyliner-shape
   cylinder-shape-p
   new-cone-shape
   cone-shape-p
   new-compound-shape
   compound-shape-p
   add-child-shape
   new-convex-hull-shape
   convex-hull-shape-p
   add-point
   ;; constraints
   delete-constraint
   new-point-2-point-constraint
   point-2-point-constraint-p
   new-hinge-constraint
   hinge-constraint-p
   set-angular-only
   get-angular-only
   enable-angular-motor
   enable-motor
   set-max-motor-impulse
   get-max-motor-impulse
   get-motor-target-velocity
   set-motor-target
   set-limit
   set-limit-complex
   get-hinge-angle
   get-lower-limit
   get-upper-limit
   new-slider-constraint
   slider-constraint-p
   get-lower-lin-limit
   set-lower-lin-limit
   get-upper-lin-limit
   set-upper-lin-limit
   get-lower-ang-limit
   set-lower-ang-limit
   get-upper-ang-limit
   set-upper-ang-limit
   get-softness-dir-lin
   get-restitution-dir-lin
   get-damping-dir-lin
   get-softness-dir-ang
   get-restitution-dir-ang
   get-damping-dir-ang
   get-softness-lim-lin
   get-restitution-lim-lin
   get-damping-lim-lin
   get-softness-lim-ang
   get-restitution-lim-ang
   get-damping-lim-ang
   get-softness-ortho-lin
   get-restitution-ortho-lin
   get-damping-ortho-lin
   get-softness-ortho-ang
   get-restitution-ortho-ang
   get-damping-ortho-ang
   set-softness-dir-lin
   set-restitution-dir-lin
   set-damping-dir-lin
   set-softness-dir-ang
   set-restitution-dir-ang
   set-damping-dir-ang
   set-softness-lim-lin
   set-restitution-lim-lin
   set-damping-lim-lin
   set-softness-lim-ang
   set-restitution-lim-ang
   set-damping-lim-ang
   set-softness-ortho-lin
   set-restitution-ortho-lin
   set-damping-ortho-lin
   set-softness-ortho-ang
   set-restitution-ortho-ang
   set-damping-ortho-ang
   set-powered-lin-motor
   get-powered-lin-motor
   set-target-lin-motor-velocity
   get-target-lin-motor-velocity
   set-max-lin-motor-force
   get-max-lin-motor-force
   set-target-ang-motor-velocity
   get-target-ang-motor-velocity
   set-max-ang-motor-force
   get-max-ang-motor-force
   get-linear-pos
   set-debug-mode
   get-debug-mode
   set-callbacks
   get-callbacks
   ;; debug-draw
   debug-draw
   draw-line
   draw-sphere
   draw-triangle
   draw-box
   draw-aabb
   draw-transform
   draw-arc
   draw-sphere-patch
   draw-contact-point
   report-error-warning
   draw-3d-text))
