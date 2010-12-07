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
  (make-instance 'cl-transforms:3d-vector
                 :x (mem-aref pointer :double 0)
                 :y (mem-aref pointer :double 1)
                 :z (mem-aref pointer :double 2)))

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
  (make-instance 'cl-transforms:quaternion
                 :x (mem-aref pointer :double 0)
                 :y (mem-aref pointer :double 1)
                 :z (mem-aref pointer :double 2)
                 :w (mem-aref pointer :double 3)))

(defmethod free-translated-object (value (type bt-quaternion) param)
  (declare (ignore param))
  (foreign-free value))
