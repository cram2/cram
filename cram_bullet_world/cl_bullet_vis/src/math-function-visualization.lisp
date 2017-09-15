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

(defclass math-function-object (display-list-mixin)
  ((width :initarg :width :reader width)
   (height :initarg :height :reader height)
   (step-size :initarg :step-size :reader step-size :initform 0.1)
   (pose :initarg :pose :reader pose :initform (cl-transforms:make-pose
                                                (cl-transforms:make-3d-vector 0 0 0)
                                                (cl-transforms:make-quaternion 0 0 0 1)))
   (fun :initarg :function :reader fun)
   (alpha :initarg :alpha :reader alpha :initform 1.0)
   (color-fun :initarg :color-fun :reader color-fun :initform (constantly (list 0.0 0.0 0.6)))))

(defmethod gl-object-transparent ((obj math-function-object))
  (< (alpha obj) 1.0))

(defmethod draw ((context gl-context) (obj math-function-object))
  (let ((points (make-hash-table :test 'equal))
        (colors (make-hash-table :test 'equal)))
    (flet ((get-point (x y)
             (with-slots (fun) obj
               (multiple-value-bind (old-value found?)
                   (gethash (list x y) points)
                 (if found?
                     old-value
                     (let ((new-value (funcall fun x y)))
                       (setf (gethash (list x y) points)
                             (when (numberp new-value)
                               (cl-transforms:make-3d-vector x y new-value))))))))
           (get-color (vec)
             (let ((x (cl-transforms:x vec))
                   (y (cl-transforms:y vec)))
               (with-slots (color-fun) obj
                 (or (gethash (list x y) colors)
                     (setf (gethash (list x y) colors)
                           (append (subseq (funcall color-fun vec) 0 3)
                                   (list (alpha obj))))))))
           (vertex (vec &optional normal)
             (let ((normal (cl-transforms:v*
                            normal
                            (/ (cl-transforms:v-norm normal)))))
               (when normal
                 (gl:normal
                  (cl-transforms:x normal)
                  (cl-transforms:y normal)
                  (cl-transforms:z normal))))
             (gl:vertex
              (cl-transforms:x vec)
              (cl-transforms:y vec)
              (cl-transforms:z vec))))
      (with-slots (width height step-size pose fun) obj
        (gl:with-pushed-matrix
          (gl:with-pushed-attrib (:current-bit :enable-bit)
            (when (gl-object-transparent obj)
              (gl:disable :cull-face))
            (gl:mult-matrix (pose->gl-matrix pose))
            (loop for x from (- (/ width 2)) to (- (/ width 2) step-size) by step-size do
              (loop for y from (- (/ height 2)) to (- (/ height 2) step-size) by step-size do
                (let ((pt-1 (get-point x y))
                      (pt-2 (get-point (+ x step-size) y))
                      (pt-3 (get-point x (+ y step-size)))
                      (pt-4 (get-point (+ x step-size) (+ y step-size))))
                  (when (and pt-1 pt-2 pt-3 pt-4)
                    (let ((n-1 (cl-transforms:cross-product
                                (cl-transforms:v- pt-2 pt-1)
                                (cl-transforms:v- pt-4 pt-1)))
                          (n-2 (cl-transforms:cross-product
                                (cl-transforms:v- pt-4 pt-1)
                                (cl-transforms:v- pt-3 pt-1))))
                      (gl:with-primitive :triangles
                        (apply #'gl:color (get-color pt-1))
                        (vertex pt-1 n-1)
                        (apply #'gl:color (get-color pt-2))
                        (vertex pt-2 n-1)
                        (apply #'gl:color (get-color pt-4))
                        (vertex pt-4 n-1)
                        (apply #'gl:color (get-color pt-1))
                        (vertex pt-1 n-2)
                        (apply #'gl:color (get-color pt-4))
                        (vertex pt-4 n-2)
                        (apply #'gl:color (get-color pt-3))
                        (vertex pt-3 n-2)))))))))))))