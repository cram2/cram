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

(defgeneric get-texture-handle (gl-context name))
(defgeneric camera-transform (gl-context)
  (:documentation "Returns the transform of the current camera as a
  CL-TRANSFORMS:TRANSFORM."))
(defgeneric light-position (gl-context)
  (:documentation "Returns the position of the light as a
  CL-TRANSFORMS:3D-VECTOR."))

(defclass gl-context ()
  ((textures :initform (make-hash-table))
   (camera-transform :initform (cl-transforms:make-pose
                                (cl-transforms:make-3d-vector 0 0 0)
                                (cl-transforms:make-quaternion 0 0 0 1))
                     :initarg :camera-transform
                     :reader camera-transform)
   (light-position :initform (cl-transforms:make-3d-vector 1 1 5)
                   :initarg :light-position
                   :reader light-position)))

(defmethod get-texture-handle ((context gl-context) (name symbol))
  (or (gethash name (slot-value context 'textures))
      (values
        (setf (gethash name (slot-value context 'textures))
              (car (gl:gen-textures 1)))
        t)))
