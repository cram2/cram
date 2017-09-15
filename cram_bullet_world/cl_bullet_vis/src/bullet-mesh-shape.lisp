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

(defclass mesh-shape-mixin ()
  ((faces :initarg :faces :reader faces)
   (smooth-shading :initarg :smooth-shading :initform nil)
   (disable-face-culling :initarg :disable-face-culling :initform nil)))

(defmethod draw ((context gl-context) (mesh mesh-shape-mixin))
  (with-slots (smooth-shading disable-face-culling) mesh
    (gl:with-pushed-attrib (:enable-bit)
      (when disable-face-culling
        (gl:disable :cull-face))
      (gl:with-primitive :triangles
        (map 'nil
             (lambda (face)
               (loop for v in (physics-utils:face-points face)
                     for n in (physics-utils:face-normals face)
                     do
                  (if smooth-shading
                      (let ((norm (cl-transforms:v-norm v)))
                        (gl:normal (/ (cl-transforms:x v) norm)
                                   (/ (cl-transforms:y v) norm)
                                   (/ (cl-transforms:z v) norm)))
                      (gl:normal (cl-transforms:x n)
                                 (cl-transforms:y n)
                                 (cl-transforms:z n)))
                  (gl:vertex (cl-transforms:x v)
                             (cl-transforms:y v)
                             (cl-transforms:z v))))
             (faces mesh))))))

(defclass box-mesh-shape (mesh-shape-mixin
                          colored-shape-mixin
                          box-shape
                          display-list-mixin)
  ())

(defclass cylinder-mesh-shape (mesh-shape-mixin
                               colored-shape-mixin
                               cylinder-shape
                               display-list-mixin)
  ())

(defclass compound-mesh-shape (mesh-shape-mixin
                               colored-shape-mixin
                               compound-shape
                               display-list-mixin)
  ())

(defclass convex-hull-mesh-shape (mesh-shape-mixin
                                  colored-shape-mixin
                                  convex-hull-shape
                                  display-list-mixin)
  ())
