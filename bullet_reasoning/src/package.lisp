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

(in-package :cl-user)

(defpackage bullet-reasoning
    (:nicknames :btr)
  (:use #:common-lisp #:crs #:bt #:bt-vis #:cut)
  (:import-from #:alexandria compose curry rcurry)
  (:export merge-bounding-boxes aabb
           *debug-window* add-debug-window
           camera width height fov-y z-near z-far camera-axis pose
           gl-setup-camera camera-transform look-at-object-rotation
           with-rendering-to-framebuffer render-to-framebuffer
           read-pixelbuffer read-depthbuffer to-png-image
           add-object generic-cup mug plate mondamin mesh
           remove-object object name rigid-bodies rigid-body-names
           rigid-body world make-object box static-plane sphere
           cylinder cone point-cloud
           bt-reasoning-world invalidate-object objects object
           bt-reasoning-world-state
           robot-object links joint-states urdf joint-state link-names
           link-pose set-robot-state-from-tf
           semantic-map-object semantic-map-geoms semantic-map-geom-names
           semantic-map-geom semantic-map
           ensure-pose ensure-vector
           object-visibility object-visibility-percentage
           object-visibility-occluding-objects flat-color-object-proxy
           calculate-object-visibility object-visible-p occluding-objects
           simulate find-objects contact-p find-all-contacts
           find-objects-in-contact poses-equal-p stable-p above-p
           find-objects-above below-p find-objects-below))
