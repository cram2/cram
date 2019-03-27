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

(in-package :location-costmap)

(defgeneric get-point (object)
  (:documentation "Returns the point that is described by
  `object'. E.g. if `object' is a pose, returns the position part.")
  (:method ((pose cl-transforms:pose))
    (cl-transforms:origin pose))
  (:method ((point cl-transforms:3d-vector))
    point))

(defun make-gauss-cost-function (mean covariance)
  (let ((mean (etypecase mean
                (list (make-array 2 :initial-contents mean))
                (array mean)
                (cl-transforms:3d-vector
                 (make-array
                  2 :initial-contents (list
                                       (float (cl-transforms:x mean) 0.0d0)
                                       (float (cl-transforms:y mean) 0.0d0))))))
        (covariance (etypecase covariance
                      (list (make-array '(2 2) :initial-contents covariance))
                      (array covariance))))
    (let ((gauss-fun (cma:gauss (cma:double-matrix-from-array covariance)
                                (cma:double-matrix-from-array mean)))
          (pos (cma:make-double-vector 2)))
      (lambda (x y)
        (setf (aref pos 0 0) (float x 0.0d0))
        (setf (aref pos 1 0) (float y 0.0d0))
        (funcall gauss-fun pos)))))

(defun make-location-cost-function (loc std-dev)
  (let ((loc (cl-transforms:origin loc)))
    (make-gauss-cost-function loc `((,(float (* std-dev std-dev) 0.0d0) 0.0d0)
                                    (0.0d0 ,(float (* std-dev std-dev)))))))

(defun make-range-cost-function (pose distance &key invert)
  "Returns a costfunction that returns 1 for every point that is not
  further than distance away from point."
  (let* ((point (get-point pose)) (z (cl-transforms:z point))
         (in-range (if invert 0.0d0 1.0d0))
         (out-range (if invert 1.0d0 0.0d0)))
    (lambda (x y)
      (if (> (cl-transforms:v-dist point (cl-transforms:make-3d-vector x y z))
             distance)
          out-range
          in-range))))

(defun make-axis-boundary-cost-function (axis boundary predicate)
  "Returns a cost function that has the value 1 if the pose is on the
respective side of `boundary'.

The value of `axis' is either :X or :Y.

`boundary' is a NUMBER.

`side' is either :left or :right. If `side' is :left, the value 1.0 is
returned for poses < `boundary', otherwise poses > `boundary' result
in a value of 1.0"

  (let ((pred (etypecase predicate
                (function predicate)
                (symbol (symbol-function predicate)))))
    (lambda (x y)
      (if (funcall
           pred
           (ecase axis
             (:x x)
             (:y y))
           boundary)
          1.0d0
          0.0d0))))

(defun make-occupancy-grid-cost-function (grid &key invert)
  (when grid
    (let ((grid (if invert
                    (invert-occupancy-grid grid)
                    grid)))
      (flet ((generator (costmap-metadata matrix)
               (declare (type cma:double-matrix matrix))
               (let ((start-x (max (origin-x grid) (origin-x costmap-metadata)))
                     (start-y (max (origin-y grid) (origin-y costmap-metadata)))
                     (end-x  (min (+ (width grid) (origin-x grid))
                                  (+ (width costmap-metadata) (origin-x costmap-metadata))))
                     (end-y (min (+ (height grid) (origin-y grid))
                                 (+ (height costmap-metadata) (origin-y costmap-metadata))))
                     (grid-matrix (grid grid)))
                 (loop for y-source-index from (map-coordinate->array-index
                                                start-y (resolution grid) (origin-y grid))
                         below (map-coordinate->array-index
                                end-y (resolution grid) (origin-y grid))
                           by (/ (resolution costmap-metadata) (resolution grid))
                       for y-destination-index from (map-coordinate->array-index
                                                     start-y (resolution costmap-metadata)
                                                     (origin-y costmap-metadata))
                         below (map-coordinate->array-index
                                end-y (resolution costmap-metadata)
                                (origin-y costmap-metadata))
                           by (/ (resolution costmap-metadata)
                                 (resolution costmap-metadata))
                       do (loop for x-source-index from (map-coordinate->array-index
                                                         start-x (resolution grid) (origin-x grid))
                                  below (map-coordinate->array-index
                                         end-x (resolution grid) (origin-x grid))
                                    by (/ (resolution costmap-metadata) (resolution grid))
                                for x-destination-index from (map-coordinate->array-index
                                                              start-x (resolution costmap-metadata)
                                                              (origin-x costmap-metadata))
                                  below (map-coordinate->array-index
                                         end-x (resolution costmap-metadata)
                                         (origin-x costmap-metadata))
                                    by (/ (resolution costmap-metadata)
                                          (resolution costmap-metadata))
                                do (setf
                                    (aref matrix
                                          (truncate y-destination-index)
                                          (truncate x-destination-index))
                                    (float
                                     (aref grid-matrix
                                           (truncate y-source-index)
                                           (truncate x-source-index))
                                     0.0d0)))
                       finally (return matrix)))))
        (make-instance 'map-costmap-generator
          :generator-function #'generator)))))

(defun make-padded-costmap-cost-function (costmap padding &key invert)
  (when (and costmap padding)
    (let* ((grid (matrix->occupancy-grid (origin-x costmap)
                                         (origin-y costmap)
                                         (resolution costmap)
                                         (get-cost-map costmap)))
           (padding-mask (make-padding-mask (round (/ padding (resolution costmap)))))
           (pred (if invert
                     (lambda (x) (eql x 0))
                     (lambda (x) (eql x 1))))
           (grid-arr (grid grid))
           (max-row (- (array-dimension grid-arr 0)
                       (truncate (/ padding (resolution costmap)))))
           (max-col (- (array-dimension grid-arr 1)
                       (truncate (/ padding (resolution costmap)))))
           (result-grid (copy-occupancy-grid grid)))
      (do ((row (truncate (/ padding (resolution costmap))) (1+ row)))
          ((>= row max-row))
        (do ((col (truncate (/ padding (resolution costmap))) (1+ col)))
            ((>= col max-col))
          (when (funcall pred (aref grid-arr row col))
            (occupancy-grid-put-mask col row result-grid padding-mask :coords-raw-p t))))
      (make-occupancy-grid-cost-function result-grid :invert invert))))

(defun rotate-point-around-another-point (x y sin-theta cos-theta
                                          rotation-point-x rotation-point-y)
  (let* ((x-translated (- x rotation-point-x))
         (y-translated (- y rotation-point-y))
         (x-rotated (- (* x-translated cos-theta) (* y-translated sin-theta)))
         (y-rotated (+ (* x-translated sin-theta) (* y-translated cos-theta)))
         (x-translated-back (+ x-rotated rotation-point-x))
         (y-translated-back (+ y-rotated rotation-point-y)))
    (list x-translated-back y-translated-back)))

(defun make-matrix-cost-function (origin-x origin-y resolution matrix &optional theta)
  "Creates a cost function that adds a matrix (i.e. a 2d bitmap) of
type CMA:DOUBLE-MATRIX into the costmap. `resolution' specifies the
resolution of `matrix', i.e. the size in meters of one
entry. `origin-x' and `origin-y' specify the position of the top left
corner (bottom right if viewed from top down, i.e. from the tip of the Z axis, looking down).
Optionally an angle `theta' can be given in radiant, if the input matrix needs to be rotated."
  (declare (type number origin-x origin-y resolution)
           (type cma:double-matrix matrix))
  (flet ((generator (costmap-metadata output-matrix)
           (declare (type cma:double-matrix output-matrix))

           ;; we first fill in an empty matrix, then add it to already existing
           ;; output-matrix, which might have values in it from before this function
           ;; this is especially important when rotating the costmap:
           ;; some values might get assigned twice to the same cell,
           ;; due to rounding up errors or so.
           ;; to avoid messing up the costmap values, we don't incf the values,
           ;; but simply setf
           ;; as we cannot setf values into the output matrix if it already has values,
           ;; we need an extra clean matrix to put the input in, and then add to the original
           (let ((empty-output-matrix
                   (cma:make-double-matrix
                    (truncate (width costmap-metadata) (resolution costmap-metadata))
                    (truncate (height costmap-metadata) (resolution costmap-metadata)))))

             ;; If we look top down on the costmap visualization,
             ;; `start' is on the right bottom corner, `end' in on the top left.
             ;; `start' and `end' are the coordinates of the overlapping rectangle
             ;; between input `matrix' and `output-matrix'.
             ;; `start' and `end' are in meters.
             (let* ((destination-origin-y
                      (origin-y costmap-metadata))
                    (destination-origin-x
                      (origin-x costmap-metadata))
                    (destination-end-y
                      (+ destination-origin-y (height costmap-metadata)))
                    (destination-end-x
                      (+ destination-origin-x (width costmap-metadata)))

                    (start-y
                      (max origin-y destination-origin-y))
                    (start-x
                      (max origin-x destination-origin-x))
                    (end-y
                      (min (array-index->map-coordinate (cma:height matrix) resolution origin-y)
                           destination-end-y))
                    (end-x
                      (min (array-index->map-coordinate (cma:width matrix) resolution origin-x)
                           destination-end-x))

                    (sin-theta (when theta (sin theta)))
                    (cos-theta (when theta (cos theta))))

               ;; due to machine precision of floating-point arithmetic,
               ;; going below a certain coordinate (<),
               ;; then converting it to array index and back,
               ;; might bring you right back at the certain coordinate (=).
               ;; that's why we're adding a little epsilon to ensure the BELOW.
               (loop for y-coordinate
                     from start-y below (- end-y 0.000001) by (resolution costmap-metadata)
                     do (loop for x-coordinate
                              from start-x below (- end-x 0.000001) by (resolution costmap-metadata)
                              do (let ((y-coordinate-maybe-rotated y-coordinate)
                                       (x-coordinate-maybe-rotated x-coordinate))
                                   (when theta
                                     (let ((new-x-and-y
                                             (rotate-point-around-another-point
                                              x-coordinate y-coordinate sin-theta cos-theta
                                              (+ origin-x
                                                 (* (/ (cma:width matrix) 2.0) resolution))
                                              (+ origin-y
                                                 (* (/ (cma:height matrix) 2.0) resolution)))))
                                      (setf y-coordinate-maybe-rotated
                                            (second new-x-and-y)
                                            x-coordinate-maybe-rotated
                                            (first new-x-and-y))
                                      (when (or (< y-coordinate-maybe-rotated destination-origin-y)
                                                (>= y-coordinate-maybe-rotated destination-end-y)
                                                (< x-coordinate-maybe-rotated destination-origin-x)
                                                (>= x-coordinate-maybe-rotated destination-end-x))
                                        (continue))))

                                   (let ((y-input-index
                                           (map-coordinate->array-index
                                            y-coordinate resolution origin-y))
                                         (x-input-index
                                           (map-coordinate->array-index
                                            x-coordinate resolution origin-x))
                                         (y-destination-index
                                           (map-coordinate->array-index
                                            y-coordinate-maybe-rotated
                                            (resolution costmap-metadata) destination-origin-y))
                                         (x-destination-index
                                           (map-coordinate->array-index
                                            x-coordinate-maybe-rotated
                                            (resolution costmap-metadata) destination-origin-x)))

                                     (setf (aref empty-output-matrix
                                                 y-destination-index x-destination-index)
                                           (aref matrix
                                                 y-input-index x-input-index)))))
                     finally (return (cma:m+ output-matrix empty-output-matrix)))))))
    (make-instance 'map-costmap-generator
      :generator-function #'generator)))

(defun make-constant-height-function (height)
  (lambda (x y)
    (declare (ignore x y))
    (list height)))
