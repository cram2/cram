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

(in-package :cl-urdf)

(defclass inertial ()
  ((origin :reader origin :initarg :origin
           :initform (cl-transforms:make-transform
                      (cl-transforms:make-3d-vector 0 0 0)
                      (cl-transforms:make-quaternion 0 0 0 1)))
   (mass :reader mass :initarg :mass :initform 0)
   (inertia :reader inertia :initarg :inertia)))

(defclass geometry () ())

(defclass box (geometry)
  ((size :initarg :size :reader size
         :documentation "The size of the box as a CL-TRANSFORMS:3D-VECTOR")))

(defclass cylinder (geometry)
  ((radius :initarg :radius :reader radius)
   (length :initarg :length :reader cylinder-length)))

(defclass sphere (geometry)
  ((radius :initarg :radius :reader radius)))

(defclass mesh (geometry)
  ((filename :initarg :filename :reader filename)
   (scale :initarg :scale :initform nil :reader scale)
   (size :initarg :size :initform nil :reader size)))

(defclass material ()
  ((name :reader name :initarg :name)
   (color :reader color :initarg :color
          :initform '(0.8 0.8 0.8 1.0))
   (texture :reader texture :initarg :texture)))

(defclass visual ()
  ((origin :reader origin :initarg :origin
           :initform (cl-transforms:make-transform
                      (cl-transforms:make-3d-vector 0 0 0)
                      (cl-transforms:make-quaternion 0 0 0 1)))
   (geomerty :reader geometry :initarg :geometry)
   (material :reader material :initarg :material
             :initform (make-instance 'material))))

(defclass collision ()
  ((origin :reader origin :initarg :origin
           :initform (cl-transforms:make-transform
                      (cl-transforms:make-3d-vector 0 0 0)
                      (cl-transforms:make-quaternion 0 0 0 1)))
   (geometry :reader geometry :initarg :geometry)))

(defclass link ()
  ((from-joint :reader from-joint :initarg :from-joint
               :initform nil
               :documentation "The joint that connects this link with
               its parent link")
   (to-joints :reader to-joints :initarg :joints
              :initform nil
              :documentation "The list of joints that connect this
              link with its children")
   (name :reader name :initarg :name)
   (intertial :reader inertial :initarg :inertial)
   (visual :reader visual :initarg :visual)
   (collision :reader collision :initarg :collision)))

