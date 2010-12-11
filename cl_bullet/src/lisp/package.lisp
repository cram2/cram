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
   
   ))
