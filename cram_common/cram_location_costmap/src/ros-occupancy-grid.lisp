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

(in-package :location-costmap)

(defun grid-value-occupied-p (val)
  (and (> val 0) (<= val 127)))

(defun grid-value-not-occupied-p (val)
  (= val 0))

(defun occupancy-grid-msg->occupancy-grid (msg &key padding invert)
  "takes a ros message, and creates an occupancy grid, where 1 means
occupied, and zero means free, or vice versa if invert is set.
Padding pads in the generated grid, not in the original grid, which
has implications for invert, meaning that free space in the original
grid becomes a larger obstacle in the generated grid. So with padding,
a grid generated without invert and one with are not inverts of each
other."
  (roslisp:with-fields ((resolution (resolution info))
                        (height (height info))
                        (width (width info))
                        (origin-x (x position origin info))
                        (origin-y (y position origin info))
                        (map-data data))
      msg
    (declare (type (simple-array t 1) map-data))
    (let* ((grid (make-occupancy-grid
                  (+ (* width resolution) (* 2 (or padding 0.0)))
                  (+ (* height resolution) (* 2 (or padding 0.0)))
                  (- origin-x (or padding 0.0))
                  (-  origin-y (or padding 0.0))
                  resolution))
           (grid-arr (grid grid))
           (i 0)
           (padding-offset (truncate (/ (or padding 0.0) resolution)))
           (max-row (round (+ height padding-offset)))
           (max-col (round (+ width padding-offset)))
           (padding-mask (when padding (make-padding-mask (round (/ padding resolution)))))
           (pred (if invert #'grid-value-not-occupied-p #'grid-value-occupied-p)))
      (declare (type occupancy-grid-data grid-arr)
               (type (or null (simple-array fixnum 2)) padding-mask))
      (do ((row padding-offset (1+ row)))
          ((>= row max-row))
        (do ((col padding-offset (1+ col)))
            ((>= col max-col))
          (when (funcall pred (aref map-data i))
            (if padding
                (occupancy-grid-put-mask col row grid padding-mask :coords-raw-p t)
                (setf (aref grid-arr row col) 1)))
          (incf i)))
      grid)))

(defun occupancy-grid-mean (map &key (invert nil))
  (roslisp:with-fields ((resolution (resolution info))
                        (width (width info))
                        (height (height info))
                        (origin-x (x position origin info))
                        (origin-y (y position origin info))
                        (map-data data))
      map
    (let ((pred (if invert #'grid-value-not-occupied-p #'grid-value-occupied-p))
          (size (length map-data)))
      (loop for y from 0 below height
            with cntr = 0
            with sum-x = 0.0
            with sum-y = 0.0
            do (loop for x from 0 below width
                     when (funcall pred (aref map-data cntr)) do
                       (setf sum-x (+ sum-x x))
                       (setf sum-y (+ sum-y y))
                     do (incf cntr))
            finally (return (cl-transforms:make-3d-vector
                             (+ (* (/ sum-x size) resolution) origin-x)
                             (+ (* (/ sum-y size) resolution) origin-y)
                             0))))))

(defun occupancy-grid-covariance (map &key (mean nil) (invert nil))
  (roslisp:with-fields ((resolution (resolution info))
                        (width (width info))
                        (height (height info))
                        (origin-x (x position origin info))
                        (origin-y (y position origin info))
                        (map-data data))
      map
    (let* ((mean (or mean (occupancy-grid-mean map :invert invert)))
           (cov-matrix (make-array '(2 2) :initial-element 0.0))
           (pred (if invert #'grid-value-not-occupied-p #'grid-value-occupied-p))
           (size (length map-data)))
      (loop for y from 0 below height
            with cntr = 0
            do (loop for x from 0 below width
                     for x-scaled = (+ (* x resolution) origin-x)
                     for y-scaled = (+ (* y resolution) origin-y)
                     when (funcall pred (aref map-data cntr)) do
                       (setf (aref cov-matrix 0 0)
                             (+ (aref cov-matrix 0 0)
                                (expt (- x-scaled (cl-transforms:x mean)) 2)))
                       (setf (aref cov-matrix 1 0)
                             (+ (aref cov-matrix 1 0)
                                (* (- x-scaled (cl-transforms:x mean))
                                   (- y-scaled (cl-transforms:y mean)))))
                       (setf (aref cov-matrix 1 1)
                             (+ (aref cov-matrix 1 1)
                                (expt (- y-scaled (cl-transforms:y mean)) 2)))
                     do (incf cntr))
            finally (progn
                      (setf (aref cov-matrix 0 0) (/ (aref cov-matrix 0 0) size))
                      (setf (aref cov-matrix 1 0) (/ (aref cov-matrix 1 0) size)) 
                      (setf (aref cov-matrix 1 1) (/ (aref cov-matrix 1 1) size))
                      (setf (aref cov-matrix 0 1) (aref cov-matrix 1 0))
                      (return cov-matrix))))))

(defun occupancy-grid-msg-metadata (map &key key)
  "Returns a list containing the map metadata. The list can be
destructured and contains the
keys :width, :height, :resolution, :origin-x and :origin-y"
  (assert map () "No occupancy grid specifed")
  (roslisp:with-fields ((resolution (resolution info))
                        (height (height info))
                        (width (width info))
                        (origin-x (x position origin info))
                        (origin-y (y position origin info)) )
      map
    (case key
      (:width (round (* width resolution)))
      (:height (round (* height resolution)))
      (:resolution resolution)
      (:origin-x origin-x)
      (:origin-y origin-y)
      (:origin (list origin-x origin-y))
      (t `(:width ,(round (* width resolution)) :height ,(round (* height resolution))
                  :resolution ,resolution
                  :origin-x ,(round origin-x) :origin-y ,(round origin-y))))))
