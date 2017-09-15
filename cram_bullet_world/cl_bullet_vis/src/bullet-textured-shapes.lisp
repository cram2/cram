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

(in-package :bt-vis)

(defvar *disable-texture-rendering* nil
  "When set to T, textures are disabled")

(defclass textured-shape-mixin ()
  ((texture :initarg :texture
            :reader texture)
   (width :initarg :width
          :reader width)
   (height :initarg :height
           :reader height)
   (texture-name :initarg :texture-name
                 :initform (gensym)
                 :reader texture-name)))

(defun bind-texture (context name bitmap width height)
  (multiple-value-bind (texture new?)
      (get-texture-handle context name)
    (gl:bind-texture :texture-2d texture)
    (when new?
      (make-mipmaps
       texture bitmap
       width height))
    texture))

(defmethod draw :around ((context gl-context) (shape textured-shape-mixin))
  (if *disable-texture-rendering*
      (call-next-method)
      (gl:with-pushed-attrib (:texture-bit :enable-bit)
        (with-slots (texture-name texture width height) shape
          (bind-texture context texture-name texture width height)
          (gl:enable :texture-2d)
          (call-next-method)))))

(defclass textured-box-shape (colored-box-shape textured-shape-mixin) ())
(defclass textured-static-plane-shape (colored-static-plane-shape textured-shape-mixin) ())
(defclass textured-sphere-shape (colored-sphere-shape textured-shape-mixin) ())
(defclass textured-cone-shape (colored-cone-shape textured-shape-mixin) ())
(defclass textured-compound-shape (colored-compound-shape textured-shape-mixin) ())
(defclass textured-convex-hull-shape (colored-convex-hull-shape textured-shape-mixin) ())
