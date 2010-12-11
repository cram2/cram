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

(in-package :cl-bullet)

(defcenum activation-state
  (:active-tag 1)
  (:island-sleeping 2)
  (:wants-deactivation 3)
  (:disable-deactivation 4)
  (:disable-simulation 5))

(defcenum collision-flags
  (:cf-static-object 1)
  (:cf-kinematic-object 2)
  (:cf-no-contact-response 4)
  (:cf-custom-material-callback 8)
  (:cf-character-object 16)
  (:cf-disable-visualize-object 32))

(defcstruct debug-draw-callbacks
  (draw-line :pointer)
  (draw-sphere :pointer)
  (draw-triangle :pointer)
  (draw-box :pointer)
  (draw-aabb :pointer)
  (draw-transform :pointer)
  (draw-arc :pointer)
  (draw-sphere-patch :pointer)
  (draw-contact-point :pointer)
  (report-error-warning :pointer)
  (draw-3d-text :pointer))

(define-foreign-type bt-3d-vector ()
  () (:actual-type :pointer :double))

(define-parse-method bt-3d-vector (&key)
  (make-instance 'bt-3d-vector))

(defmethod translate-to-foreign ((value cl-transforms:3d-vector) (type bt-3d-vector))
  (let ((native-vector (foreign-alloc :double :count 3)))
    (setf (mem-aref native-vector :double 0) (coerce (cl-transforms:x value) 'double-float))
    (setf (mem-aref native-vector :double 1) (coerce (cl-transforms:y value) 'double-float))
    (setf (mem-aref native-vector :double 2) (coerce (cl-transforms:z value) 'double-float))
    native-vector))

(defmethod translate-from-foreign (pointer (type bt-3d-vector))
  (cl-transforms:make-3d-vector
   (mem-aref pointer :double 0)
   (mem-aref pointer :double 1)
   (mem-aref pointer :double 2)))

(defmethod free-translated-object (value (type bt-3d-vector) param)
  (declare (ignore param))
  (foreign-free value))

(define-foreign-type bt-quaternion ()
  () (:actual-type :pointer :double))

(define-parse-method bt-quaternion (&key)
  (make-instance 'bt-quaternion))

(defmethod translate-to-foreign ((value cl-transforms:quaternion) (type bt-quaternion))
  (let ((native-vector (foreign-alloc :double :count 4)))
    (setf (mem-aref native-vector :double 0) (coerce (cl-transforms:x value) 'double-float))
    (setf (mem-aref native-vector :double 1) (coerce (cl-transforms:y value) 'double-float))
    (setf (mem-aref native-vector :double 2) (coerce (cl-transforms:z value) 'double-float))
    (setf (mem-aref native-vector :double 3) (coerce (cl-transforms:w value) 'double-float))
    native-vector))

(defmethod translate-from-foreign (pointer (type bt-quaternion))
  (cl-transforms:make-quaternion
   (mem-aref pointer :double 0)
   (mem-aref pointer :double 1)
   (mem-aref pointer :double 2)
   (mem-aref pointer :double 3)))

(defmethod free-translated-object (value (type bt-quaternion) param)
  (declare (ignore param))
  (foreign-free value))

(define-foreign-type bt-transform ()
  () (:actual-type :pointer :double))

(define-parse-method bt-transform (&key)
  (make-instance 'bt-transform))

(defmethod translate-to-foreign ((value cl-transforms:transform) (type bt-transform))
  (let ((native-vector (foreign-alloc :double :count 7))
        (translation (cl-transforms:translation value))
        (rotation (cl-transforms:rotation value)))
    (setf (mem-aref native-vector :double 0) (coerce (cl-transforms:x translation) 'double-float))
    (setf (mem-aref native-vector :double 1) (coerce (cl-transforms:y translation) 'double-float))
    (setf (mem-aref native-vector :double 2) (coerce (cl-transforms:z translation) 'double-float))
    (setf (mem-aref native-vector :double 3) (coerce (cl-transforms:x rotation) 'double-float))
    (setf (mem-aref native-vector :double 4) (coerce (cl-transforms:y rotation) 'double-float))
    (setf (mem-aref native-vector :double 5) (coerce (cl-transforms:z rotation) 'double-float))
    (setf (mem-aref native-vector :double 6) (coerce (cl-transforms:w rotation) 'double-float))
    native-vector))

(defmethod translate-to-foreign ((value cl-transforms:pose) (type bt-transform))
  (let ((native-vector (foreign-alloc :double :count 7))
        (translation (cl-transforms:origin value))
        (rotation (cl-transforms:orientation value)))
    (setf (mem-aref native-vector :double 0) (coerce (cl-transforms:x translation) 'double-float))
    (setf (mem-aref native-vector :double 1) (coerce (cl-transforms:y translation) 'double-float))
    (setf (mem-aref native-vector :double 2) (coerce (cl-transforms:z translation) 'double-float))
    (setf (mem-aref native-vector :double 3) (coerce (cl-transforms:x rotation) 'double-float))
    (setf (mem-aref native-vector :double 4) (coerce (cl-transforms:y rotation) 'double-float))
    (setf (mem-aref native-vector :double 5) (coerce (cl-transforms:z rotation) 'double-float))
    (setf (mem-aref native-vector :double 6) (coerce (cl-transforms:w rotation) 'double-float))
    native-vector))

(defmethod translate-from-foreign (pointer (type bt-transform))
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector
    (mem-aref pointer :double 0)
    (mem-aref pointer :double 1)
    (mem-aref pointer :double 2))
   (cl-transforms:make-quaternion
    (mem-aref pointer :double 3)
    (mem-aref pointer :double 4)
    (mem-aref pointer :double 5)
    (mem-aref pointer :double 6))))

(defmethod free-translated-object (value (type bt-transform) param)
  (declare (ignore param))
  (foreign-free value))