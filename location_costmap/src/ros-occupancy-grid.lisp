;;; KiPla - Cognitive kitchen planner and coordinator
;;; Copyright (C) 2010 by Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;;
;;; This program is free software; you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation; either version 3 of the License, or
;;; (at your option) any later version.
;;;
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;;
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :location-costmap)

(defun grid-value-occupied-p (val)
  (and (> val 0) (<= val 127)))

(defun grid-value-not-occupied-p (val)
  (eql val 0))

(defun occupancy-grid-msg->occupancy-grid (msg &key padding invert)
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
