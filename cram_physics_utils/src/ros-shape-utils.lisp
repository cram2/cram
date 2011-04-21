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

(in-package :physics-utils)

(defun point-msg->3d-vector (msg)
  (roslisp:with-fields (x y z) msg
    (cl-transforms:make-3d-vector
     x y z)))

(defun shape-msg->points (msg)
  (roslisp:with-fields (shape) msg
    (roslisp:with-fields (type vertices)
        shape
      (assert (or (eql type 3) (eql type 4)) ()
              "This method requires point type 4")
      (map 'vector #'point-msg->3d-vector vertices))))

(defun shape-msg->mesh (msg)
  (roslisp:with-fields (shape) msg
    (roslisp:with-fields (type triangles vertices)
        shape
      (assert (eql type 3) () "We require a mesh in the message.")
      (physics-utils:make-3d-model
       :vertices (physics-utils::remove-identical-vertices
                  (map 'vector #'point-msg->3d-vector vertices))
       :faces (let ((result (make-array (/ (length vertices) 3))))
                (loop for i from 0 below (length vertices) by 3
                      for j from 0 do
                        (let* ((point-1 (point-msg->3d-vector (aref vertices (aref triangles i))))
                               (point-2 (point-msg->3d-vector (aref vertices (aref triangles (+ i 1)))))
                               (point-3 (point-msg->3d-vector (aref vertices (aref triangles (+ i 2)))))
                               (normal (cl-transforms:cross-product (cl-transforms:v-
                                                                     point-2 point-1)
                                                                    (cl-transforms:v-
                                                                     point-3 point-1)))
                               (normal-normalized (cl-transforms:v*
                                                   normal (/ (cl-transforms:v-norm normal)))))
                          (setf (aref result j)
                                (physics-utils:make-face
                                 :normals (list normal-normalized normal-normalized normal-normalized)
                                 :points (list point-1 point-2 point-3))))
                      finally (return result)))))))
