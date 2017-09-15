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

(defvar *draw-bounding-boxes* nil)

(defmethod draw ((context gl-context) (body rigid-body))
  (gl:with-pushed-matrix
    (gl:mult-matrix (pose->gl-matrix (pose body)))
    (draw context (collision-shape body)))
  (when *draw-bounding-boxes*
    (draw context (aabb body))))

(defmethod draw ((context gl-context) (bounding-box bounding-box))
  (gl:with-pushed-matrix
    (gl:translate
     (cl-transforms:x (bounding-box-center bounding-box))
     (cl-transforms:y (bounding-box-center bounding-box))
     (cl-transforms:z (bounding-box-center bounding-box)))
    (gl:scale
     (cl-transforms:x (bounding-box-dimensions bounding-box))
     (cl-transforms:y (bounding-box-dimensions bounding-box))
     (cl-transforms:z (bounding-box-dimensions bounding-box)))
    (gl:with-pushed-attrib (:current-bit)
      (gl:color 1 0 1)
      (glut:wire-cube 1))))
