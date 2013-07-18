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

(in-package :bullet)

(defclass collision-shape (foreign-class) ())

(defmethod foreign-class-free-fun ((obj collision-shape))
  #'delete-collision-shape)

(defclass box-shape (collision-shape)
  ((half-extents :initarg :half-extents :reader half-extents)))

(defmethod foreign-class-alloc ((obj box-shape) &key half-extents &allow-other-keys)
  (new-box-shape half-extents))

(defclass static-plane-shape (collision-shape)
  ((normal :initarg :normal :reader normal)
   (constant :initarg :constant :reader constant)))

(defmethod foreign-class-alloc ((obj static-plane-shape) &key
                                normal constant &allow-other-keys)
  (new-static-plane-shape normal (coerce constant 'double-float)))

(defclass sphere-shape (collision-shape)
  ((radius :initarg :radius :reader radius)))

(defmethod foreign-class-alloc ((obj sphere-shape) &key radius &allow-other-keys)
  (new-sphere-shape (coerce radius 'double-float)))

(defclass cylinder-shape (collision-shape)
  ((half-extents :initarg :half-extents :reader half-extents)))

(defmethod foreign-class-alloc ((obj cylinder-shape) &key half-extents &allow-other-keys)
  (new-cylinder-shape half-extents))

(defclass cone-shape (collision-shape)
  ((radius :initarg :radius :reader radius)
   (height :initarg :height :reader height)))

(defmethod foreign-class-alloc ((obj cone-shape) &key radius height &allow-other-keys)
  (new-cone-shape (coerce radius 'double-float) (coerce height 'double-float)))

(defclass compound-shape (collision-shape)
  ((children :initform nil)))

(defmethod foreign-class-alloc ((obj compound-shape) &key &allow-other-keys)
  (new-compound-shape))

(defgeneric add-child-shape (obj pose shape)
  (:method ((obj compound-shape) pose shape)
    (push `(,shape . ,pose) (slot-value obj 'children))
    (let ((transform (cl-transforms:reference-transform pose)))
      (cffi-add-child-shape
       (foreign-obj obj)
       (cl-transforms:translation transform)
       (cl-transforms:rotation transform)
       (foreign-obj shape)))))

(defgeneric children (obj)
  (:method ((obj compound-shape))
    (mapcar #'car (slot-value obj 'children))))

(defgeneric child-pose (obj child)
  (:method ((obj compound-shape) (child collision-shape))
    (cdr (assoc child (slot-value obj 'children)))))

(defclass convex-hull-shape (collision-shape)
  ((points :initform nil :initarg :points :reader points)))

(defmethod foreign-class-alloc ((obj convex-hull-shape) &key points &allow-other-keys)
  (new-convex-hull-shape points))

(defgeneric add-point (obj point)
  (:method ((obj convex-hull-shape) (point cl-transforms:3d-vector))
    (push point (slot-value obj 'points))
    (cffi-add-point (foreign-obj obj) point)))
