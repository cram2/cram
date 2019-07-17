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

(in-package :cram-btr-visibility-costmap)

(defparameter *vertex-shader*
  "#version 110

void main() {
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
")

(defparameter *fragment-shader*
  "#version 110

void main(void)
{
  gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
}
")

(defclass white-shader-program (bt-vis:shader-program) ()
  (:default-initargs
   :vertex-shader-source *vertex-shader*
   :fragment-shader-source *fragment-shader*))

(defclass shader-camera (camera) ()
  (:default-initargs :fov-y 90))

(defmethod btr:gl-execute-with-camera ((camera shader-camera) (function function))
  (let ((shader-program (make-instance 'white-shader-program)))
    (unwind-protect
         (bt-vis:call-with-shader-program shader-program #'call-next-method)
      (bt-vis:release-shader-program shader-program))))

(defun normalize-depth-buffer (camera depth-buffer)
  (declare (type simple-array depth-buffer))
  (flet ((normalize (angle depth-value)
           (with-slots (z-far z-near) camera
             (let ((z-value (/ (/ (* z-far z-near) (- z-near z-far))
                               (- depth-value (/ z-far (- z-far z-near))))))
               (assert (< z-value z-far))
               (/ z-value (cos angle))))))
    (with-slots (fov-y width height) camera
      (let ((normalized-depth-image (make-array (list height width)
                                                :element-type 'double-float)))
        (declare (type simple-array normalized-depth-image))
        (loop
          with index = 0
          for y-index from 0 below height
          do (loop for x-index from 0 below width
                   for angle from (cma:degrees->radians (/ fov-y -2)) by (/ pi 180)
                   do (setf (aref normalized-depth-image y-index x-index)
                            (float (normalize angle (aref depth-buffer index))
                                   0.0d0))
                      (incf index))
          finally (return normalized-depth-image))))))

(defun render-depth-maps (drawable point width height)
  "Renders 4 depth maps, each with a foy of 90 degrees and size
`width' x `height'."
  (declare (type cl-transforms:3d-vector point)
           (type fixnum width height))
  (let ((rendering-context (btr:get-rendering-context width height))
        (base-orientation (cl-transforms:make-quaternion
                           -0.4999999701976776d0 0.4999999701976776d0
                           -0.4999999701976776d0 0.4999999701976776d0))
        (pose-transform (cl-transforms:make-transform
                         point (cl-transforms:make-identity-rotation))))
    (bt-vis:with-gl-context rendering-context
      (gl:with-pushed-attrib (:enable-bit :color-buffer-bit :depth-buffer-bit)
        (gl:disable :lighting)
        (%gl:clear-color 1.0 1.0 1.0 1.0)
        (gl:depth-func :lequal)
        (gl:enable :depth-test)
        (loop for i below 4
              for camera = (make-instance 'shader-camera
                             :pose (cl-transforms:transform-pose
                                    pose-transform
                                    (cl-transforms:make-pose
                                     (cl-transforms:make-identity-vector)
                                     (cl-transforms:q*
                                      (cl-transforms:euler->quaternion
                                       :az (* (mod (+ i 1) 4) (/ pi 2)))
                                      base-orientation)))
                             :width width
                             :height height)
              for depth-image = (normalize-depth-buffer
                                 camera
                                 (second (btr:render-to-framebuffer
                                          rendering-context drawable camera
                                          :get-pixelbuffer nil :get-depthbuffer t
                                          :mirror t)))
              collecting depth-image
              ;; do (png::encode-file (with-slots (fov-y z-far) camera
              ;;                        (let ((normalized (make-array (array-total-size depth-image) :element-type 'double-float))
              ;;                              (max-value (* z-far (/ (cos (cma:degrees->radians (/ fov-y 2)))))))
              ;;                          (dotimes (i (array-total-size depth-image))
              ;;                            (setf (aref normalized i)
              ;;                                  (min
              ;;                                   (/ (row-major-aref depth-image i)
              ;;                                      max-value)
              ;;                                   1.0d0)))
              ;;                          (to-png-image width height normalized :gray)))
              ;;                      (format nil "/tmp/foo-~a.png" i))
              )))))

(defun calculate-map-column-indices (x y pixels)
  "Returns two values, the column for getting the distance value in a
square depth image of size `pixels' and the index of the depth image."
  (declare (type fixnum x y pixels))
  (let ((pixels/2 (truncate pixels 2)))
    (flet ((calculate-column (x y)
             (let ((x-normalized (- x pixels/2))
                   (y-normalized (abs (- pixels/2 y))))
               (cond ((eql y-normalized 0) pixels/2)
                     (t (+ (truncate (* x-normalized pixels/2) y-normalized)
                           pixels/2))))))
      (cond ((and (< y pixels/2) (>= x y) (< x (- pixels y)))
             (values (calculate-column x y)
                     0))
            ((and (< x pixels/2) (> y x) (< y (- pixels x)))
             (values (calculate-column (- pixels y 1) x)
                     1))
            ((and (> y pixels/2) (< x y) (> x (- pixels y 1)))
             (values
              (calculate-column (- pixels x 1) y)
              2))
            (t
             (values
              (calculate-column y (- pixels x 1))
              3))))))

(defun calculate-column-ranges (minimal-camera-height maximal-camera-height size resolution)
  (let* ((size/2 (/ size 2))
         (entries (round size resolution))
         (ranges (make-array (list entries entries 2))))
    (declare (type simple-array ranges))
    (loop for y-index below entries
          for y from (- size/2) by resolution
          do (loop for x-index below entries
                   for x from (- size/2) by resolution
                   for distance = (sqrt (+ (* x x) (* y y)))
                   do (setf (aref ranges y-index x-index 0)
                            (atan minimal-camera-height distance))
                      (setf (aref ranges y-index x-index 1)
                            (atan maximal-camera-height distance)))
          finally (return ranges))))

(defun calculate-visibility-costmap-matrix
    (drawable point camera-minimal-height camera-maximal-height size resolution)
  (let* ((pixels (round size resolution))
         (costmap (make-array (list pixels pixels)
                              :element-type 'double-float))
         (depth-maps (map 'vector #'identity
                          (render-depth-maps drawable point pixels pixels)))
         (size/2 (/ size 2))
         (column-ranges (calculate-column-ranges
                         (- camera-minimal-height (cl-transforms:z point))
                         (- camera-maximal-height (cl-transforms:z point))
                         size resolution)))
    (declare (simple-array costmap))
    (dotimes (y pixels costmap)
      (dotimes (x pixels)
        (multiple-value-bind (column-index map-index)
            (calculate-map-column-indices x y pixels)
          (let ((maximal-value 0))
            (loop for row from (max
                                (- pixels
                                   (+ (/ pixels 2)
                                      (truncate
                                       (* (aref column-ranges y x 1) (/ pixels 2)))))
                                0)
                    to (min
                        (- pixels
                           (+ (/ pixels 2)
                              (truncate
                               (* (aref column-ranges y x 0) (/ pixels 2)))))
                        (1- pixels))
                  for distance-from-origin = (sqrt
                                              (+ (expt
                                                  (- (* x resolution) size/2) 2)
                                                 (expt
                                                  (- (* y resolution) size/2) 2)))
                  do (when (and
                            (< distance-from-origin size/2)
                            (< distance-from-origin
                               (* (aref (aref depth-maps map-index) row column-index))))
                       (incf (aref costmap (- pixels y 1) x) 1.0d0))
                     (incf maximal-value))
            (when (> maximal-value 0)
              (setf (aref costmap (- pixels y 1) x)
                    (/ (aref costmap (- pixels y 1) x)
                       maximal-value)))))))))

(defun visibility-costmap
    (drawable pose camera-minimal-height camera-maximal-height size resolution)
  (costmap:make-matrix-cost-function
   (- (cl-transforms:x (cl-transforms:origin pose)) (/ size 2))
   (- (cl-transforms:y (cl-transforms:origin pose)) (/ size 2))
   resolution (calculate-visibility-costmap-matrix
               drawable
               (cl-transforms:origin pose)
               camera-minimal-height camera-maximal-height size resolution)))

(defun make-location-visibility-costmap
    (world location robot-name camera-minimal-height camera-maximal-height size resolution)
  (declare (type btr:bt-reasoning-world world)
           (type desig:location-designator location)
           (type (or symbol string) robot-name)
           (type number camera-maximal-height camera-minimal-height size resolution))
  (let* ((robot (btr:object world robot-name))
         (objects-to-render (remove robot (objects world))))
    (visibility-costmap
     (btr:make-drawable-list :drawables objects-to-render)
     (desig:reference (desig:current-desig location)) camera-minimal-height camera-maximal-height
     size resolution)))

(defun make-object-visibility-costmap
    (world object-name robot-name camera-minimal-height camera-maximal-height size resolution)
  (declare (type btr:bt-reasoning-world world)
           (type (or symbol string) object-name robot-name)
           (type number camera-maximal-height camera-minimal-height size resolution))
  (let* ((object (btr:object world object-name))
         (robot (btr:object world robot-name))
         (objects-to-render (remove-if (lambda (current-object)
                                         (or (eq object current-object)
                                             (eq robot current-object)))
                                       (objects world))))
    (visibility-costmap
     (btr:make-drawable-list :drawables objects-to-render)
     (btr:pose object) camera-minimal-height camera-maximal-height
     size resolution)))
