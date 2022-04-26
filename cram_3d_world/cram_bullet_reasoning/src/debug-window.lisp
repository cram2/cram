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

(defvar *debug-window* nil)
(defvar *debug-window-lock* (sb-thread:make-mutex))
(defvar *current-costmap-function* nil "A math function object for visualizing costmaps.")
(defvar *current-costmap-sample* nil "A red sphere for visualizing costmap samples.")
(defvar *vis-axes* nil "An associative list of ((ID . (X Y Z)) ...) couples,
storing axes of a coordinate frame for visualizing poses.")
(defparameter *vis-axis-width* 0.01 "In meters")
(defparameter *vis-axis-length* 0.3 "In meters")
(defparameter *vis-axis-alpha* 1.0 "Color alpha value of frame visualization.")
(defparameter *costmap-z* 0.005)
(defparameter *costmap-tilt* (cl-transforms:make-quaternion 0 0 0 1))
(defparameter *costmap-sample-dims*
  (cl-transforms:make-3d-vector
   0.02 0.02 0.02)
  "Half extents in meters.")

(defun add-debug-window (world)
  (sb-thread:with-mutex (*debug-window-lock*)
    (cond ((not (and *debug-window* (not (closed *debug-window*))))
           (setf *debug-window* (make-instance 'bullet-world-window
                                  :world world
                                  :camera-transform (cl-transforms:make-transform
                                                     (cl-transforms:make-3d-vector -5 0 3)
                                                     (cl-transforms:axis-angle->quaternion
                                                      (cl-transforms:make-3d-vector 0 1 0)
                                                      (/ pi 8)))
                                  :light-position (cl-transforms:make-3d-vector -1.8 -2.0 5.0)))
           (sb-thread:make-thread
            (lambda () (glut:display-window *debug-window*))
            :name "Debug window"))
          ((not (eq world (world *debug-window*)))
           (setf (world *debug-window*)
                 world)))
    (when (hidden *debug-window*)
      (show-window *debug-window*))))

(defun costmap-color-fun (val)
  (let ((segment (* val 4)))
    (cond ((< segment 1.0)
           (list 0.0 (* val 4) 1.0))
          ((< segment 2.0)
           (list 0.0 1.0 (- 1.0 (* (- val 0.25) 4))))
          ((< segment 3.0)
           (list (* (- val 0.5) 4) 1.0 0.0))
          ((< segment 4.0)
           (list 1.0 (- 1.0 (* (- val 0.75) 4)) 0.0))
          (t (list 1.0 0.0 0.0)))))

(defun clear-costmap-vis-object (&key (costmap t) (sample t) (axes t))
  (sb-thread:with-mutex (*debug-window-lock*)
    (when (and costmap *current-costmap-function* *debug-window*)
      (setf (gl-objects *debug-window*)
            (remove *current-costmap-function* (gl-objects *debug-window*)))
      (setf *current-costmap-function* nil))
    (when (and sample *current-costmap-sample* *debug-window*)
      (setf (gl-objects *debug-window*)
            (remove *current-costmap-sample* (gl-objects *debug-window*)))
      (setf *current-costmap-sample* nil))
    (when (and axes *vis-axes* *debug-window*)
      (mapcar (lambda (id-axes-pair)
                (setf (gl-objects *debug-window*)
                      (reduce (lambda (gl-objects-list axis-object)
                                (remove axis-object gl-objects-list))
                              (cdr id-axes-pair)
                              :initial-value (gl-objects *debug-window*))))
              *vis-axes*)
      (setf *vis-axes* nil))))



(defun add-costmap-function-object (costmap)
  (sb-thread:with-mutex (*debug-window-lock*)
    (when (and *current-costmap-function* *debug-window*)
      (setf (gl-objects *debug-window*)
            (remove *current-costmap-function* (gl-objects *debug-window*))))
    (when costmap
      (let* ((map-array (costmap:get-cost-map costmap))
             (max-val (loop for y from 0 below (array-dimension map-array 0)
                            maximizing (loop for x from 0 below (array-dimension map-array 1)
                                             maximizing (aref map-array y x)))))
        (declare (type cma:double-matrix map-array))
        (flet ((costmap-function (x y)
                 (let ((val (/ (costmap:get-map-value costmap x y) max-val)))
                   (when (> val costmap:*costmap-valid-solution-threshold*)
                     val)))
               (costmap-height-function (x y)
                 (let ((val (costmap:generate-height costmap x y)))
                   (+ val *costmap-z*))))
          (setf *current-costmap-function*
                (make-instance 'math-function-object
                  :width (costmap:grid-width costmap)
                  :height (costmap:grid-height costmap)
                  :alpha 0.5
                  :color-fun #'costmap-color-fun
                  :pose (cl-transforms:make-pose
                         (cl-transforms:make-3d-vector
                          (+ (costmap:origin-x costmap)
                             (/ (costmap:grid-width costmap) 2))
                          (+ (costmap:origin-y costmap)
                             (/ (costmap:grid-height costmap) 2))
                          *costmap-z*)
                         *costmap-tilt*)
                  :function #'costmap-function
                  :height-function #'costmap-height-function
                  :step-size (costmap:resolution costmap)))))
      (when *debug-window*
        (push *current-costmap-function* (gl-objects *debug-window*))))))


(defun add-costmap-sample-object (point)
  (sb-thread:with-mutex (*debug-window-lock*)
    (when (and *current-costmap-sample* *debug-window*)
      (setf (gl-objects *debug-window*)
            (remove *current-costmap-sample* (gl-objects *debug-window*))))
    (when (and point (typep point 'cl-transforms:3d-vector))
      (setf *current-costmap-sample*
            (make-instance 'rigid-body
              :pose (cl-transforms:make-pose
                     point
                     (cl-transforms:make-identity-rotation))
              :collision-shape (make-instance 'colored-box-shape
                                 :half-extents *costmap-sample-dims*
                                 :color '(1.0 0.0 0.0 0.5))))
      (when *debug-window*
        (push *current-costmap-sample* (gl-objects *debug-window*))))))


(defgeneric add-vis-axis-object (object-or-pose &key length width id)
  (:documentation "Spawn a coordinate frame at a given pose or at btr:object origin.
It is built from 3 rigid bodies of primitive box shape.
`Length' specified the length of a single axis.
`Id' can be used if one wants to visualize multiple frames at the same time.")

  (:method ((object-name symbol)
            &key (length *vis-axis-length*) (width *vis-axis-width*) (id 0))
    (add-vis-axis-object
     (pose (btr:object *current-bullet-world* object-name))
     :length length :width width :id id))

  (:method ((pose cl-transforms:pose)
            &key (length *vis-axis-length*) (width *vis-axis-width*) (id 0))
    (sb-thread:with-mutex (*debug-window-lock*)
      (when (and *vis-axes* *debug-window* (assoc id *vis-axes*))
        (setf (gl-objects *debug-window*)
              (reduce (lambda (gl-objects-list axis-object)
                        (remove axis-object gl-objects-list))
                      (cdr (assoc id *vis-axes*))
                      :initial-value (gl-objects *debug-window*))))

      (let ((length/2 (/ length 2))
            (width/2 (/ width 2))
            (object-transform (cl-transforms:pose->transform pose)))

        ;; spawn the objects
        (flet ((make-axis-rigid-body (position-list half-extents-list color)
                 (make-instance 'rigid-body
                   :pose (cl-transforms:transform->pose
                          ;; make the center of axis not in the center of pose
                          ;; but offset, such that it's a corner and not a cross
                          (cl-transforms:transform*
                           object-transform
                           (cl-transforms:make-transform
                            (apply #'cl-transforms:make-3d-vector position-list)
                            (cl-transforms:make-identity-rotation))))
                   :collision-shape (make-instance 'colored-box-shape
                                      :half-extents (apply #'cl-transforms:make-3d-vector
                                                           half-extents-list)
                                      :color color))))

          (let ((new-axes-list
                  (list (make-axis-rigid-body
                         `(,length/2 0 0)
                         `(,length/2 ,width/2 ,width/2)
                         `(0.5 0 0 ,*vis-axis-alpha*))
                        (make-axis-rigid-body
                         `(0 ,length/2 0)
                         `(,width/2 ,length/2 ,width/2)
                         `(0 0.5 0 ,*vis-axis-alpha*))
                        (make-axis-rigid-body
                         `(0 0 ,length/2)
                         `(,width/2 ,width/2 ,length/2)
                         `(0 0 0.5 ,*vis-axis-alpha*)))))
            (if (assoc id *vis-axes*)
                (rplacd (assoc id *vis-axes*) new-axes-list)
                (push (cons id new-axes-list) *vis-axes*))))

        (when *debug-window*
          (mapc (lambda (obj) (push obj (gl-objects *debug-window*)))
                (cdr (assoc id *vis-axes*))))))))




(defmethod costmap:on-visualize-costmap opengl ((map costmap:location-costmap))
  (add-costmap-function-object map))

(defmethod costmap:on-visualize-costmap-sample opengl ((point cl-transforms:3d-vector))
  (add-costmap-sample-object point))
