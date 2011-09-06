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

(in-package :semantic-map-costmap)

(defparameter *board-thickness* 0.035)

(defun obj-z-value (object &optional (tag :on))
  (let ((origin (cl-transforms:origin (sem-map-utils:pose object)))
        (dimensions (sem-map-utils:dimensions object)))
    (ecase tag
      (:on (+ (cl-transforms:z origin) (/ (cl-transforms:z dimensions) 2)))
      ;; We assume objects are box-like and `in' means on the bottom
      (:in (+ (- (cl-transforms:z origin) (/ (cl-transforms:z dimensions) 2))
              *board-thickness*)))))

(defun get-aabb (&rest points)
  (loop for p in points
        for x = (cl-transforms:x p)
        for y = (cl-transforms:y p)
        for z = (cl-transforms:z p)
        minimizing x into min-x
        maximizing x into max-x
        minimizing y into min-y
        maximizing y into max-y
        minimizing z into min-z
        maximizing z into max-z
        finally
     (return (list (cl-transforms:make-3d-vector min-x min-y min-z)
                   (cl-transforms:make-3d-vector max-x max-y max-z)))))

(defun inside-aabb (min max pt)
  "Checks if `pt' lies in the axis-alligned bounding box specified by
  the two points `min' and `max'"
  (let ((x (cl-transforms:x pt))
        (y (cl-transforms:y pt))
        (z (cl-transforms:z pt)))
    (and (>= x (cl-transforms:x min))
         (<= x (cl-transforms:x max))
         (>= y (cl-transforms:y min))
         (<= y (cl-transforms:y max))
         (>= z (cl-transforms:z min))
         (<= z (cl-transforms:z max)))))

(defun 2d-object-bb (dimensions &optional pose)
  "Returns the 2-dimensional aabb of a semantic-map object"
  (let* ((transform (when pose (cl-transforms:reference-transform pose)))
         (dimensions/2 (cl-transforms:v* dimensions 0.5))
         (bb-pts (list (cl-transforms:v* dimensions/2 -1)
                       dimensions/2)))
    (apply
     #'get-aabb
     (if transform
         (mapcar (lambda (pt)
                   (cl-transforms:transform-point transform pt))
                 bb-pts)
         bb-pts))))

(defun point-on-object (object point)
  (let* ((bb (2d-object-bb (sem-map-utils:dimensions object)))
         (transform (cl-transforms:pose->transform (sem-map-utils:pose object)))
         (pt (cl-transforms:make-3d-vector
              (cl-transforms:x point)
              (cl-transforms:y point)
              (cl-transforms:z
               (cl-transforms:translation transform)))))
    (funcall #'inside-aabb (first bb) (second bb)
             (cl-transforms:transform-point
              (cl-transforms:transform-inv transform)
              pt))))

(defun make-semantic-map-obj-generator (object &key (padding 0.0))
  (declare (type sem-map-utils:semantic-map-geom object))
  (let* ((transform (cl-transforms:pose->transform (sem-map-utils:pose object)))
         (dimensions (cl-transforms:v+
                      (sem-map-utils:dimensions object)
                      (cl-transforms:make-3d-vector padding padding padding)))
         (pt->obj-transform (cl-transforms:transform-inv transform))
         ;; Since our map is 2d we need to select a z value for our
         ;; point. We just use the pose's z value since it should be
         ;; inside the object.
         (z-value (cl-transforms:z (cl-transforms:translation transform))))
    (destructuring-bind ((obj-min obj-max)
                         (local-min local-max))
        (list (2d-object-bb dimensions transform)
              (2d-object-bb dimensions))
      ;; For performance reasons, we first check if the point is
      ;; inside the object's bounding box in map and then check if it
      ;; really is inside the object.
      (lambda (x y)
        (let ((pt (cl-transforms:make-3d-vector x y z-value)))
          (when (and (inside-aabb obj-min obj-max pt)
                     (inside-aabb local-min local-max (cl-transforms:transform-point
                                                       pt->obj-transform pt)))
            1.0))))))

(defun make-semantic-map-costmap (objects &key (invert nil) (padding 0.0))
  "Generates a semantic-map costmap for all `objects'. `objects' is a
list of SEM-MAP-UTILS:SEMANTIC-MAP-GEOMs"
  (let ((functions (mapcar (alexandria:rcurry #'make-semantic-map-obj-generator
                                              :padding padding)
                           (cut:force-ll objects))))
    (if invert
        (lambda (x y)
          (if (some (alexandria:rcurry #'funcall x y) functions)
              0.0 1.0))
        (lambda (x y)
          (if (some (alexandria:rcurry #'funcall x y) functions)
              1.0 0.0)))))

(defun make-on-cost-function (object)
  (let ((transform (cl-transforms:pose->transform (sem-map-utils:pose object)))
        (dimensions (sem-map-utils:dimensions object)))
    (let* ((cov-x-axis (cl-transforms:rotate
                        (cl-transforms:rotation transform)
                        (cl-transforms:make-3d-vector (cl-transforms:x dimensions) 0 0)))
           (cov-y-axis (cl-transforms:rotate
                        (cl-transforms:rotation transform)
                        (cl-transforms:make-3d-vector 0 (cl-transforms:y dimensions) 0)))
           (cov (points-cov (list cov-x-axis cov-y-axis)
                            (cl-transforms:make-3d-vector 0 0 0))))
      (make-gauss-cost-function
       (make-array 2 :element-type 'double-float
                     :initial-contents
                     (list (float (cl-transforms:x (cl-transforms:translation transform)) 0.0d0)
                           (float (cl-transforms:y (cl-transforms:translation transform)) 0.0d0)))
       (2d-cov cov)))))

(defun make-semantic-map-height-function (objects &optional (type-tag :on))
  (lambda (x y)
    (let ((heights (loop for obj in (cut:force-ll objects)
                         when (point-on-object obj (cl-transforms:make-3d-vector x y 0))
                           collecting (float (obj-z-value obj type-tag) 0.0d0))))
      (alexandria:random-elt heights))))

(defun make-constant-height-function (height)
  (lambda (x y)
    (declare (ignore x y))
    height))
