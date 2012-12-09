;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :bt-vis)

(defclass reachability-map-visualization (display-list-mixin)
  ((reachability-map :initarg :reachability-map :reader reachability-map)
   (pose :initarg :pose :reader pose)))

(defclass reachability-map-object (reachability-map-visualization) ())

(defclass inverse-reachability-map-object (reachability-map-visualization) ())

(defmethod gl-object-transparent ((obj reachability-map-visualization))
  nil)

(defun draw-reachability-map-entry (reachability-map-object cell-pose value-function)
  (with-slots (reachability-map) reachability-map-object
    (gl:with-pushed-matrix
      (gl:mult-matrix (pose->gl-matrix cell-pose))
      (let ((reachability-value (funcall value-function reachability-map cell-pose))
            (resolution (pr2-reachability-costmap:resolution reachability-map)))
        (when (> reachability-value 0.0)
          (apply #'gl:color (reachability-map-color-function reachability-value))
          (gl:scale (cl-transforms:x resolution)
                    (cl-transforms:y resolution)
                    (cl-transforms:z resolution))
          ;; (glut:solid-cube 1.0)
          (glut:solid-sphere 0.5 10 10))))))

(defun reachability-map-color-function (value)
  (let ((segment (* value 4))
        (alpha 1))
    (cond ((< segment 1.0)
           (list 0.0 (* value 4) 1.0 alpha))
          ((< segment 2.0)
           (list 0.0 1.0 (- 1.0 (* (- value 0.25) 4)) alpha))
          ((< segment 3.0)
           (list (* (- value 0.5) 4) 1.0 0.0 alpha))
          ((< segment 4.0)
           (list 1.0 (- 1.0 (* (- value 0.75) 4)) 0.0 alpha))
          (t (list 1.0 0.0 0.0)))))

(defun draw-reachability-map-grid
    (reachability-map-object value-function minimum maximum
     &key (draw-p (constantly t)))
  "Draws all entries in the `reachability-map' at `pose'. Iterates
  from `minimum' to `maximum'. `value-function' is used to calculate
  the value that is put into the color function."
  (with-slots (reachability-map pose) reachability-map-object
    (gl:with-pushed-matrix
      (gl:with-pushed-attrib (:current-bit)
        (gl:mult-matrix (pose->gl-matrix pose))
        (with-simple-restart (continue "continue")
          (loop for y from (cl-transforms:y minimum)
                  to (cl-transforms:y maximum)
                    by (cl-transforms:y (pr2-reachability-costmap:resolution reachability-map))
                do (loop for x from (cl-transforms:x minimum)
                           to (cl-transforms:x maximum)
                             by (cl-transforms:x (pr2-reachability-costmap:resolution reachability-map))
                         do (loop for z from (cl-transforms:z minimum)
                                    to (cl-transforms:z maximum)
                                      by (cl-transforms:z (pr2-reachability-costmap:resolution reachability-map))
                                  when (funcall draw-p x y z)
                                    do (draw-reachability-map-entry
                                        reachability-map-object
                                        (cl-transforms:make-pose
                                         (cl-transforms:make-3d-vector x y z)
                                         (cl-transforms:make-identity-rotation))
                                        value-function)))))))))

(defmethod draw ((context gl-context) (object reachability-map-object))
  (draw-reachability-map-grid
   object #'pr2-reachability-costmap:pose-reachability
   (pr2-reachability-costmap:minimum (reachability-map object))
   (pr2-reachability-costmap:maximum (reachability-map object))))

(defmethod draw ((context gl-context) (object inverse-reachability-map-object))
  (let ((minimum (pr2-reachability-costmap:inverse-map-origin (reachability-map object)))
        (maximum (cl-transforms:v+
                  (pr2-reachability-costmap:inverse-map-origin (reachability-map object))
                  (pr2-reachability-costmap:inverse-map-size (reachability-map object)))))
    (draw-reachability-map-grid
     object #'pr2-reachability-costmap:inverse-pose-reachability
     minimum maximum
     :draw-p (lambda (x y z)
               (declare (ignorable x y z))
               (and (not (and (< x 0) (< y 0))))))))
