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

(defvar *collision-shape-color-overwrite* nil
  "When this variable is set, the actual color of the collision shape
  is overwritten and the value of *COLLISION-SHAPE-COLOR-OVERWRITE* is
  returned by COLLISION-SHAPE-COLOR")

(defclass colored-shape-mixin ()
  ((color :initarg :color
          :initform '(0.8 0.8 0.8 1.0)
          :reader shape-color)))

(defmethod draw :around ((context gl-context) (shape colored-shape-mixin))
  (gl:with-pushed-attrib (:current-bit)
    (apply #'gl:color
           (or *collision-shape-color-overwrite*
               (shape-color shape)))
    (call-next-method)))

(defmethod collision-shape-color ((shape colored-shape-mixin))
  (shape-color shape))

(defclass colored-box-shape (box-shape colored-shape-mixin) ())
(defclass colored-cylinder-shape (cylinder-shape colored-shape-mixin) ())
(defclass colored-static-plane-shape (static-plane-shape colored-shape-mixin) ())
(defclass colored-sphere-shape (sphere-shape colored-shape-mixin) ())
(defclass colored-cone-shape (cone-shape colored-shape-mixin) ())
(defclass colored-compound-shape (compound-shape colored-shape-mixin) ())
(defclass colored-convex-hull-shape (convex-hull-shape colored-shape-mixin) ())
