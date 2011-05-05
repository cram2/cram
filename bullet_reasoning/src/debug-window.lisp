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
(defvar *current-costmap-function* nil)

(defun add-debug-window (world)
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
               world))))

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

(defun add-costmap-function-object (costmap &optional (z 0.0))
  (when *current-costmap-function*
    (setf (gl-objects *debug-window*)
          (remove *current-costmap-function* (gl-objects *debug-window*))))
  (when costmap
    (let* ((map-array (location-costmap:get-cost-map costmap))
           (max-val (loop for y from 0 below (array-dimension map-array 1)
                          maximizing (loop for x from 0 below (array-dimension map-array 1)
                                           maximizing (aref map-array y x)))))
      (declare (type cma:double-matrix map-array))
      (flet ((costmap-function (x y)
           (let ((val (/ (location-costmap:get-map-value costmap x y) max-val)))
             (when (> val 0.01)
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
                                     z)
                                    (cl-transforms:make-quaternion 0 0 0 1))
                             :function #'costmap-function
                             :step-size 0.05))))
    (push *current-costmap-function* (gl-objects *debug-window*))))
