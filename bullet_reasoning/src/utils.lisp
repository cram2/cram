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

(in-package :btr)

(defun ensure-vector (vector)
  (etypecase vector
    (list (apply #'cl-transforms:make-3d-vector vector))
    (cl-transforms:3d-vector vector)))

(defun ensure-pose (pose)
  (etypecase pose
    (list (destructuring-bind
                ((x y z) (ax ay az aw))
              pose
            (cl-transforms:make-pose
             (cl-transforms:make-3d-vector x y z)
             (cl-transforms:make-quaternion ax ay az aw))))
    (cl-transforms:pose pose)))

(defun lazy-cross-product (&rest sets)
  (if (car sets)
      (let ((cp (apply #'lazy-cross-product (cdr sets))))
        (lazy-mapcan (lambda (e)
                       (lazy-mapcar (curry #'cons e) cp))
                     (car sets)))
      (list nil)))

(defun diagonal->matrix (diagonal)
  (let* ((diagonal (map 'vector #'identity diagonal))
         (dim (length diagonal))
         (result (make-array (list dim dim))))
    (dotimes (i dim result)
      (setf (aref result i i) (elt diagonal i)))))
