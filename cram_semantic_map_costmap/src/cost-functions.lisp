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

(declaim (inline inside-aabb))
(defun inside-aabb (min max pt)
  "Checks if `pt' lies in the axis-alligned bounding box specified by
  the two points `min' and `max'"
  (let ((x (cl-transforms:x pt))
        (y (cl-transforms:y pt))
        (z (cl-transforms:z pt)))
    (declare (type double-float x y z))
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
         (bb-pts (mapcar (lambda (v)
                           (destructuring-bind (x y z) v
                             (cl-transforms:make-3d-vector
                              (* (cl-transforms:x dimensions/2) x)
                              (* (cl-transforms:y dimensions/2) y)
                              (* (cl-transforms:z dimensions/2) z))))
                         '((-1 -1 -1) (-1 -1 1) (-1 1 -1) (-1 1 1)
                           (1 -1 -1) (1 -1 1) (1 1 -1) (1 1 1)))))
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
    (inside-aabb (first bb) (second bb)
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

(defun make-semantic-map-object-costmap-generator (object &key (padding 0.0))
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
      (flet ((generator-function (costmap-metadata result)
               (with-slots (origin-x origin-y resolution) costmap-metadata
                 ;; For performance reasons, we first check if the point is
                 ;; inside the object's bounding box in map and then check if it
                 ;; really is inside the object.
                 (let ((min-index-x (map-coordinate->array-index
                                     (cl-transforms:x obj-min)
                                     resolution origin-x))
                       (max-index-x (map-coordinate->array-index
                                     (cl-transforms:x obj-max)
                                     resolution origin-x))
                       (min-index-y (map-coordinate->array-index
                                     (cl-transforms:y obj-min)
                                     resolution origin-y))
                       (max-index-y (map-coordinate->array-index
                                     (cl-transforms:y obj-max)
                                     resolution origin-y)))
                   (loop for y-index from min-index-y to max-index-y
                         for y from (- (cl-transforms:y obj-min) resolution)
                           by resolution do
                             (loop for x-index from min-index-x to max-index-x
                                   for x from (- (cl-transforms:x obj-min) resolution)
                                     by resolution do
                                       (when (inside-aabb
                                              local-min local-max
                                              (cl-transforms:transform-point
                                               pt->obj-transform
                                               (cl-transforms:make-3d-vector
                                                x y z-value)))
                                         (setf (aref result y-index x-index) 1.0d0))))))
               result))
        #'generator-function))))

(defun make-semantic-object-center-generator (object padding)
  (declare (type sem-map-utils:semantic-map-geom object))
  (let* ((transform (cl-transforms:pose->transform (sem-map-utils:pose object)))
         (dimensions (cl-transforms:v-
                      (sem-map-utils:dimensions object)
                      (cl-transforms:make-3d-vector padding padding padding)))
         (obj-pose (sem-map-utils:pose object))
         (pt->obj-transform (cl-transforms:transform-inv transform))
         (z-value (cl-transforms:z (cl-transforms:translation transform))))
    (destructuring-bind ((obj-min obj-max)
                         (local-min local-max))
        (list (2d-object-bb dimensions transform)
              (2d-object-bb dimensions))
      (flet ((generator-function (costmap-metadata result)
               (with-slots (origin-x origin-y resolution) costmap-metadata
                 (let ((min-index-x (map-coordinate->array-index
                                         (cl-transforms:x obj-min)
                                         resolution origin-x))
                       (max-index-x (map-coordinate->array-index
                                         (cl-transforms:x obj-max)
                                         resolution origin-x))
                       (min-index-y (map-coordinate->array-index
                                         (cl-transforms:y obj-min)
                                         resolution origin-y))
                       (max-index-y (map-coordinate->array-index
                                         (cl-transforms:y obj-max)
                                         resolution origin-y)))
                   (let* ((mean 0.7d0)
                          (x-min (cl-transforms:x obj-min))
                          (x-max (cl-transforms:x obj-max))
                          (y-min (cl-transforms:y obj-min))
                          (y-max (cl-transforms:y obj-max))
                          (x-len (- x-max x-min))
                          (y-len (- y-max y-min))
                          (x-dominant (> x-len y-len))
                          (poses
                            (cond (x-dominant
                                   (list
                                    (cl-transforms:make-pose
                                     (cl-transforms:make-3d-vector
                                      (+ (cl-transforms:x (cl-transforms:origin obj-pose))
                                         (/ y-len 2))
                                      (+ (cl-transforms:y (cl-transforms:origin obj-pose))
                                         (/ y-len 2))
                                      0.0d0)
                                     (cl-transforms:make-identity-rotation))))
                                  (t (list
                                      (cl-transforms:make-pose
                                       (cl-transforms:make-3d-vector
                                        (+ x-min
                                           (/ x-len 2))
                                        (+ y-min
                                           (/ x-len 2))
                                        0.0d0)
                                       (cl-transforms:make-identity-rotation))
                                      (cl-transforms:make-pose
                                       (cl-transforms:make-3d-vector
                                        (+ x-min
                                           (/ x-len 2))
                                        (- y-max
                                           (/ x-len 2))
                                        0.0d0)
                                       (cl-transforms:make-identity-rotation))))))
                          (meancovs (mapcar
                                     (lambda (pose)
                                       (location-costmap::2d-pose-covariance
                                        (list pose) mean))
                                     poses))
                          (gaussians (mapcar (lambda (mean-cov)
                                               (make-gauss-cost-function
                                                (first mean-cov)
                                                (second mean-cov)))
                                             meancovs)))
                     (loop for y-index from min-index-y to max-index-y
                           for y from (- (cl-transforms:y obj-min) resolution)
                             by resolution do
                               (loop for x-index from min-index-x to max-index-x
                                     for x from (- (cl-transforms:x
                                                    obj-min) resolution)
                                       by resolution do
                                         (when (inside-aabb
                                                local-min local-max
                                                (cl-transforms:transform-point
                                                 pt->obj-transform
                                                 (cl-transforms:make-3d-vector
                                                  x y z-value)))
                                           (setf
                                            (aref result
                                                  y-index
                                                  x-index)
                                            (loop for gauss in gaussians
                                                  maximizing
                                                  (cond (x-dominant
                                                         (cond ((or
                                                                 (< x (+ x-min (/ y-len 2)))
                                                                 (> x (- x-max (/ y-len 2))))
                                                                (funcall gauss x y))
                                                         (t (funcall gauss (+ x-min
                                                                              (/ y-len 2))
                                                                     y))))
                                                        (t
                                                         (cond ((or
                                                                 (< y (+ y-min (/ x-len 2)))
                                                                 (> y (- y-max (/ x-len 2))))
                                                                (funcall gauss x y))
                                                         (t (funcall gauss x
                                                                     (+ y-min
                                                                        (/ x-len 2)))))))))))))))
               result))
        #'generator-function))))

(defun make-semantic-object-center-costmap (objects padding)
  (let ((costmap-generators
          (mapcar (lambda (object)
                    (make-semantic-object-center-generator object padding))
                  (remove-if-not (lambda (object)
                                   (typep object 'sem-map-utils:semantic-map-geom))
                                 (cut:force-ll objects)))))
    (flet ((generator (costmap-metadata matrix)
             (declare (type cma:double-matrix matrix))
             (dolist (generator costmap-generators matrix)
               (setf matrix (funcall generator costmap-metadata matrix)))))
      (make-instance 'map-costmap-generator
                     :generator-function #'generator))))

(defun make-semantic-map-costmap (objects &key invert (padding 0.0))
  "Generates a semantic-map costmap for all `objects'. `objects' is a
list of SEM-MAP-UTILS:SEMANTIC-MAP-GEOMs"
  (let ((costmap-generators (mapcar (lambda (object)
                                      (make-semantic-map-object-costmap-generator
                                       object :padding padding))
                                    (remove-if-not (lambda (object)
                                                     (typep object 'sem-map-utils:semantic-map-geom))
                                                   (cut:force-ll objects)))))
    (flet ((invert-matrix (matrix)
             (declare (type cma:double-matrix matrix))
             (dotimes (row (cma:height matrix) matrix)
               (dotimes (column (cma:width matrix))
                 (if (eql (aref matrix row column) 0.0d0)
                     (setf (aref matrix row column) 1.0d0)
                     (setf (aref matrix row column) 0.0d0)))))
           (generator (costmap-metadata matrix)
             (declare (type cma:double-matrix matrix))
             (dolist (generator costmap-generators matrix)
               (setf matrix (funcall generator costmap-metadata matrix)))))
      (make-instance 'map-costmap-generator
        :generator-function (if invert
                                (alexandria:compose #'invert-matrix #'generator)
                                #'generator)))))

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
    (loop for obj in (remove-if-not (lambda (object)
                                      (typep object 'sem-map-utils:semantic-map-geom))
                                    (cut:force-ll objects))
          when (point-on-object obj (cl-transforms:make-3d-vector x y 0))
            collecting (float (obj-z-value obj type-tag) 0.0d0))))
