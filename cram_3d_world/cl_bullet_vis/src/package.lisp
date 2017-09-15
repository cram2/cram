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

(defpackage cl-bullet-vis
    (:nicknames :bt-vis)
  (:use #:common-lisp #:bullet)
  (:import-from #:physics-utils
                event-queue post-event get-next-event)
  (:export bullet-world-window world camera-transform
           closed draw close-window
           hidden show-window hide-window
           init-camera set-camera with-gl-context
           with-rendering-lock
           get-texture-handle camera-transform light-position
           gl-context pixmap-gl-context
           bullet-world-gl-context ; <- used in tests
           bullet-world-pixmap-renderer *background-color*
           collision-shape-color *collision-shape-color-overwrite*
           colored-shape-mixin colored-static-plane-shape
           colored-box-shape colored-cylinder-shape colored-sphere-shape
           colored-cone-shape colored-compound-shape colored-convex-hull-shape
           *disable-texture-rendering*
           texture-str->bitmap textured-shape-mixin
           textured-static-plane-shape textured-box-shape
           textured-sphere-shape textured-cone-shape
           textured-compound-shape textured-convex-hull-shape
           *force-smooth-shading*
           box-mesh-shape cylinder-mesh-shape
           compound-mesh-shape convex-hull-mesh-shape
           matrix->gl-matrix pose->gl-matrix transform->gl-matrix
           force-redraw
           read-pixels-float
           math-function-object
           gl-objects *draw-bounding-boxes*
           shader-program vertex-shader fragment-shader call-with-shader-program
           set-program-uniform release-shader-program))
