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

(defun calculate-aabb (points)
  "Returns two values, a CL-TRANSFORMS:3D-VECTOR describing the size
  of the axis alligned bounding box of the 3D-MODEL `model' and
  another CL-TRANSFORMS:3D-VECTOR containing the center of the box."
  (let ((extrema
         (reduce (lambda (extrema point)
                   (setf (aref extrema 0) (min (aref extrema 0)
                                               (cl-transforms:x point)))
                   (setf (aref extrema 1) (min (aref extrema 1)
                                               (cl-transforms:y point)))
                   (setf (aref extrema 2) (min (aref extrema 2)
                                               (cl-transforms:z point)))
                   (setf (aref extrema 3) (max (aref extrema 3)
                                               (cl-transforms:x point)))
                   (setf (aref extrema 4) (max (aref extrema 4)
                                               (cl-transforms:y point)))
                   (setf (aref extrema 5) (max (aref extrema 5)
                                               (cl-transforms:z point)))
                   extrema)
                 points
                 :start 1
                 :initial-value (vector
                                 (cl-transforms:x (elt points 0))
                                 (cl-transforms:y (elt points 0))
                                 (cl-transforms:z (elt points 0))
                                 (cl-transforms:x (elt points 0))
                                 (cl-transforms:y (elt points 0))
                                 (cl-transforms:z (elt points 0))))))
    (values (cl-transforms:make-3d-vector
             (- (aref extrema 3) (aref extrema 0))
             (- (aref extrema 4) (aref extrema 1))
             (- (aref extrema 5) (aref extrema 2)))
            (cl-transforms:v*
             (cl-transforms:make-3d-vector
              (+ (aref extrema 3) (aref extrema 0))
              (+ (aref extrema 4) (aref extrema 1))
              (+ (aref extrema 5) (aref extrema 2)))
             0.5))))

(defun scale-3d-model (model scale)
  "Returns a new model scaled by the scaling-factor `scale'. `scale'
  can either be a 3d-vector or a number."
  (declare (type 3d-model model))
  (declare (type (or real cl-transforms:3d-vector) scale))
  (let ((scale (etypecase scale
                 (cl-transforms:3d-vector scale)
                 (number (cl-transforms:make-3d-vector
                          scale scale scale)))))
    (if (and (= (cl-transforms:x scale) 1)
             (= (cl-transforms:y scale) 1)
             (= (cl-transforms:z scale) 1))
        model
        (flet ((scale-vector (v s)
                 (cl-transforms:make-3d-vector
                  (* (cl-transforms:x v) (cl-transforms:x s))
                  (* (cl-transforms:y v) (cl-transforms:y s))
                  (* (cl-transforms:z v) (cl-transforms:z s)))))
          (make-3d-model
           :vertices (map 'vector
                          (lambda (v) (scale-vector v scale))
                          (3d-model-vertices model))
           :faces (map 'vector
                       (lambda (f)
                         (make-face
                          :points (mapcar (lambda (v)
                                            (scale-vector v scale))
                                          (face-points f))
                          :normals (face-normals f)))
                       (3d-model-faces model)))))))

(defun resize-3d-model (model size)
  "Returns a new model that fits into the aabb given by `size'. `size'
  must be a 3d-vector."
  (declare (type 3d-model model))
  (declare (type cl-transforms:3d-vector size))
  (let* ((aabb (calculate-aabb (3d-model-vertices model)))
         (scale (cl-transforms:make-3d-vector
                 (/ (cl-transforms:x size) (cl-transforms:x aabb))
                 (/ (cl-transforms:y size) (cl-transforms:y aabb))
                 (/ (cl-transforms:z size) (cl-transforms:z aabb)))))
    (scale-3d-model model scale)))

(defun transform-3d-model (model transform)
  "Returns a new model that is translated and rotated according to
`transform'."
  (declare (type 3d-model model))
  (declare (type cl-transforms:transform transform))
  (make-3d-model
   :vertices (map (type-of (3d-model-vertices model))
                  (lambda (point)
                    (cl-transforms:transform-point transform point))
                  (3d-model-vertices model))
   :faces (map (type-of (3d-model-faces model))
               (lambda (face)
                 (make-face
                  :points (mapcar (lambda (point)
                                    (cl-transforms:transform-point
                                     transform point))
                                  (face-points face))
                  :normals (mapcar (lambda (normal)
                                     (cl-transforms:rotate
                                      (cl-transforms:rotation transform)
                                      normal))
                                   (face-normals face))))
               (3d-model-faces model))))
