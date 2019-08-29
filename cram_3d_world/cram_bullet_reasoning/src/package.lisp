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

(defpackage cram-bullet-reasoning
  (:nicknames :btr)
  (:use #:common-lisp #:prolog #:cl-bullet #:bt-vis #:cut
        #:cram-robot-interfaces #:cl-transforms-stamped #:cram-tf)
  (:import-from #:alexandria compose curry rcurry with-gensyms copy-hash-table)
  (:import-from #:desig designator-groundings)
  (:shadowing-import-from #:cl-bullet points pose)
  (:shadow copy-object)
  (:export *current-bullet-world* *current-timeline* *visibility-threshold*
           merge-bounding-boxes aabb calculate-bb-dims
           with-stored-world *debug-window*
           add-debug-window add-costmap-function-object add-vis-axis-object
           add-costmap-sample-object clear-costmap-vis-object
           camera width
           height fov-y z-near z-far pose gl-execute-with-camera
           camera-transform look-at-object-rotation
           with-rendering-to-framebuffer render-to-framebuffer
           get-rendering-context read-pixelbuffer read-depthbuffer to-png-image
           add-object generic-cup item mesh
           remove-object object
           object-type item-type name rigid-bodies
           rigid-body-names rigid-body world make-object box
           static-plane sphere cylinder cone point-cloud
           cutlery fork knife mug plate mondamin pot bowl sugar-box apple orange
           cereal spatula pancake pancake-maker
           bt-reasoning-world invalidate-object objects object %object
           bt-reasoning-world-state robot-object links joint-states
           assert joint-state urdf joint-names joint-state link-names
           link-pose set-robot-state-from-tf
           semantic-map-object ensure-pose ensure-vector object-visibility
           semantic-map container semantic-map-part semantic-map-part-type
           semantic-map-part-pose object-visibility-percentage
           object-visibility-occluding-objects flat-color-object-proxy
           calculate-object-visibility object-visible-p
           occluding-objects simulate find-objects contact-p
           find-all-contacts find-objects-in-contact
           object-pose-different
           stable-p above-p find-objects-above below-p
           find-objects-below bullet-world object
           retract step simulate-realtime object-pose object-bottom-pose
           object-pose-on
           contact stable stable-items
           object-not-in-collision ik-solution-not-in-collision
           link-contacts supported-by above below visible visible-from
           occluding-objects occluding-object valid-grasp grasp side reachable
           object-grasp
           point-reachable pose-reachable blocking debug-window
           debug-costmap head-pointing-at with-current-bullet-world prolog-?w ?w
           ;; reach-pose-ik reach-object-ik point-reachable-p reach-point-ik
           ;; object-reachable-p pose-reachable-p calculate-orientation-in-robot
           ;; calculate-object-tool-length
           set-robot-state-from-joints
           calculate-pan-tilt
           init-ros-object-database clear-bullet-world
           ros-household-object execute open close ;; reach-ik-solution
           attached attached-objects object-attached
           attach-object detach-object detach-all-objects
           item-dimensions
           add-objects-to-mesh-list
           make-joint-state-message open-object close-object
           set-articulated-object-joint-position
           with-world copied-world with-copied-world
           obj-pose-on obj-poses-on flat-color-object-proxy drawable-list
           drawable-list-drawables make-drawable-list
           ;; robot-model-utils
           #:get-robot-object #:get-robot-name
           #:get-environment-object
           #:robot-colliding-objects-without-attached
           ;; temporal-reasoning
           event make-event timeline timeline-init timeline-advance
           timeline-current-world-state timeline-lookup
           holds-in-world with-timeline))
