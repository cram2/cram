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

(defgeneric draw-collision-shape (shape)
  (:documentation "Draws a collision shape with opengl"))

(defmethod draw-collision-shape ((box box-shape))
  (let ((size-x (* 2 (cl-transforms:x (half-extents box))))
        (size-y (* 2 (cl-transforms:y (half-extents box))))
        (size-z (* 2 (cl-transforms:z (half-extents box)))))
    (gl:with-pushed-matrix
      (gl:scale size-x size-y size-z)
      (glut:solid-cube 1))))

(defmethod draw-collision-shape ((plane static-plane-shape))
  (let ((normal (normal plane))
        (constant (constant plane)))
    (gl:with-pushed-matrix
      (gl:translate
       (* (cl-transforms:x normal) constant)
       (* (cl-transforms:y normal) constant)
       (* (cl-transforms:z normal) constant))
      (let* ((z-axis (cl-transforms:make-3d-vector 0 0 1))
             (rotation-axis (cl-transforms:cross-product z-axis normal)))
        (gl:rotate
         (* 180 (/ (acos (/ (cl-transforms:dot-product normal z-axis)
                            (cl-transforms:v-norm normal)))
                   pi))
         (cl-transforms:x rotation-axis)
         (cl-transforms:y rotation-axis)
         (cl-transforms:z rotation-axis)))
      (gl:disable :cull-face)
      (gl:with-primitive :quads
        (gl:normal 0 0 1)
        (gl:vertex -100 -100)
        (gl:vertex 100 -100)
        (gl:vertex 100 100)
        (gl:vertex -100 100))
      (gl:enable :cull-face))))

(defmethod draw-collision-shape ((sphere sphere-shape))
  (glut:solid-sphere (radius sphere) 50 50))

(defmethod draw-collision-shape ((cone cone-shape))
  (glut:solid-cone (radius cone) (height cone) 50 50))

(defmethod draw-collision-shape ((shape compound-shape))
  (dolist (child (children shape))
    (gl:with-pushed-matrix
      (gl:mult-matrix
       (pose->gl-matrix (child-pose shape child)))
      (draw-collision-shape child))))

(defmethod draw-collision-shape ((hull convex-hull-shape))
  (gl:with-primitive :points
    (dolist (point (points hull))
      (gl:vertex
       (cl-transforms:x point)
       (cl-transforms:y point)
       (cl-transforms:z point)))))
