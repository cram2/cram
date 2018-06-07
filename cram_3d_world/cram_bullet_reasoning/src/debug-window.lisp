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
(defvar *current-costmap-function* nil)
(defvar *current-costmap-sample* nil)
(defparameter *costmap-z* 0.0)
(defparameter *costmap-tilt* (cl-transforms:make-quaternion 0 0 0 1))

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

(defun costmap-color-fun (vec)
  (let ((val (cl-transforms:z vec)))
    (let ((segment (* val 4)))
      (cond ((< segment 1.0)
             (list 0.0 (* val 4) 1.0))
            ((< segment 2.0)
             (list 0.0 1.0 (- 1.0 (* (- val 0.25) 4))))
            ((< segment 3.0)
             (list (* (- val 0.5) 4) 1.0 0.0))
            ((< segment 4.0)
             (list 1.0 (- 1.0 (* (- val 0.75) 4)) 0.0))
            (t (list 1.0 0.0 0.0))))))

(defun clear-costmap-vis-object ()
  (sb-thread:with-mutex (*debug-window-lock*)
    (when (and *current-costmap-function* *debug-window*)
      (setf (gl-objects *debug-window*)
            (remove *current-costmap-function* (gl-objects *debug-window*)))
      (setf *current-costmap-function* nil))
    (when (and *current-costmap-sample* *debug-window*)
      (setf (gl-objects *debug-window*)
            (remove *current-costmap-sample* (gl-objects *debug-window*)))
      (setf *current-costmap-sample* nil))))

(defun add-costmap-function-object (costmap)
  (sb-thread:with-mutex (*debug-window-lock*)
    (when (and *current-costmap-function* *debug-window*)
      (setf (gl-objects *debug-window*)
            (remove *current-costmap-function* (gl-objects *debug-window*))))
    (when costmap
      (let* ((map-array (location-costmap:get-cost-map costmap))
             (max-val (loop for y from 0 below (array-dimension map-array 0)
                            maximizing (loop for x from 0 below (array-dimension map-array 1)
                                             maximizing (aref map-array y x)))))
        (declare (type cma:double-matrix map-array))
        (flet ((costmap-function (x y)
                 (let ((val (/ (location-costmap:get-map-value costmap x y) max-val)))
                   (when (> val location-costmap:*costmap-valid-solution-threshold*)
                     val))))
          (setf *current-costmap-function*
                (make-instance 'math-function-object
                  :width (location-costmap:grid-width costmap)
                  :height (location-costmap:grid-height costmap)
                  :alpha 0.5 :color-fun #'costmap-color-fun
                  :pose (cl-transforms:make-pose
                         (cl-transforms:make-3d-vector
                          (+ (location-costmap:origin-x costmap)
                             (/ (location-costmap:grid-width costmap) 2))
                          (+ (location-costmap:origin-y costmap)
                             (/ (location-costmap:grid-height costmap) 2))
                          *costmap-z*)
                         *costmap-tilt*)
                  :function #'costmap-function
                  :step-size (location-costmap:resolution costmap)))))
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
              :collision-shape (make-instance 'colored-sphere-shape
                                 :radius 0.05
                                 :color '(1.0 0.0 0.0 0.5))))
      (when *debug-window*
        (push *current-costmap-sample* (gl-objects *debug-window*))))))


(defmethod costmap:on-visualize-costmap opengl ((map costmap:location-costmap))
  (add-costmap-function-object map))

(defmethod costmap:on-visualize-costmap-sample opengl ((point cl-transforms:3d-vector))
  (add-costmap-sample-object point))
