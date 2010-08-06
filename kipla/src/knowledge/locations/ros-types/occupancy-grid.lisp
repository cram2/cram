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

(in-package :kipla-reasoning)

(defun grid-value-occupied-p (val)
  (and (> val 0) (<= val 127)))

(defun grid-value-not-occupied-p (val)
  (eql val 0))

(defun make-occupancy-grid-cost-function (map &key (invert nil) (padding nil))
  "Returns a cost-function to be used in a LOCATION-COST-MAP. It
returns 1 for all entries in the ros map that are free for sure and 0
otherwise."
  (flet ((extend-map (2d-map padding)
           (let* ((padding (round padding))
                  (new-map (make-array (list (+ (array-dimension 2d-map 0) (* 2 padding))
                                             (+ (array-dimension 2d-map 1) (* 2 padding)))
                                       :element-type (array-element-type 2d-map)
                                       :initial-element 0))
                  (max-y (- (array-dimension new-map 0) padding))
                  (max-x (- (array-dimension new-map 1) padding)))
             (loop for y from padding below max-y
                   for y-old from 0
                   do
                (loop for x from padding below max-x
                      for x-old from 0
                      do (setf (aref new-map y x) (aref 2d-map y-old x-old))))
             new-map)))
    (roslisp:with-fields ((resolution (resolution info))
                          (height (height info))
                          (width (width info))
                          (origin-x (x position origin info))
                          (origin-y (y position origin info))
                          (map-data data))
        map
      (let* ((coord-scaling-factor (/ resolution))
             (origin-x (if padding (- origin-x padding) origin-x))
             (origin-y (if padding (- origin-y padding) origin-y))
             (padding (when padding (round (/ padding resolution))))
             (padding-mask (when padding (make-inflation-mask (* 2 padding) 1)))
             (pred (if invert #'grid-value-not-occupied-p #'grid-value-occupied-p))
             (map-2d (if padding
                         (extend-map (make-array `(,height ,width) :displaced-to map-data) padding)
                         (make-array `(,height ,width) :displaced-to map-data)))
             (width (if padding (round (+ width (* 2 padding))) width))
             (height (if padding (round (+ height (* 2 padding))) height)))
        (lambda (x y)
          (let ((x_ (truncate (* (- x origin-x) coord-scaling-factor)))
                (y_ (truncate (* (- y origin-y) coord-scaling-factor))))
            (cond ((or (< x_ 0) (< y_ 0) (>= x_ width) (>= y_ height))
                   (if invert 1.0 0.0))
                  ((and 
                    (funcall pred (aref map-2d y_ x_))
                    (if padding
                        (not (point-in-range-mask-p map-2d x_ y_ padding-mask))
                        t))
                   1.0)
                  (t 0.0))))))))

(defun make-inflation-mask (size value)
  (let ((mask (make-array `(,size ,size) :element-type 'fixnum))
        (size/2 (/ size 2))
        (dist^2 (expt (/ size 2) 2)))
    (loop for y from 0 below size do
      (loop for x from 0 below size do
        (if (<= (+ (expt (- x size/2) 2) (expt (- y size/2) 2))
                dist^2)
            (setf (aref mask y x) value)
            (setf (aref mask y x) 0))))
    mask))

(defun point-in-range-mask-p (map x y range-mask)
  (destructuring-bind ((array-height array-width) . (mask-height mask-width))
      (cons (array-dimensions map)
            (array-dimensions range-mask))
    (let* ((mask-width/2 (round (/ mask-width 2)))
           (mask-height/2 (round (/ mask-height 2)))
           (start-x (if (< (- x mask-width/2) 0)
                        x mask-width/2))
           (end-x (if (> (+ x mask-width/2) array-width)
                      (- array-width x)
                      mask-width/2))
           (start-y (if (< (- y mask-height/2) 0)
                        y mask-height/2))
           (end-y (if (> (+ y mask-height/2) array-height)
                      (- array-height y)
                      mask-height/2)))
      (do ((y-i (- y start-y) (+ y-i 1))
           (mask-y (- mask-height/2 start-y) (+ mask-y 1)))
          ((>= y-i (+ y end-y)))
        (do ((x-i (- x start-x) (+ x-i 1))
             (mask-x (- mask-width/2 start-x) (+ mask-x 1)))
            ((>= x-i (+ x end-x)))
          (let ((mask-value (aref range-mask mask-y mask-x))
                (map-value (aref map y-i x-i)))
            (when (and (> map-value 0) (> mask-value 0))
              (return-from point-in-range-mask-p t)))))))
  nil)

(defun inflate-map (map padding &key (invert nil))
  "Inflates the occupied space inside the map. Free space with
  distance <= padding to occupied space gets marked as occupied."
  (roslisp:with-fields ((resolution (resolution info))
                        (width (width info))
                        (height (height info))
                        (header header)
                        (info info)
                        (map-data data))
      map
    (let ((map-2d (make-array `(,height ,width)
                              :displaced-to map-data))
          (new-map-2d (make-array `(,width ,height)
                                  :element-type (array-element-type map-data)))
          (inflation-mask (make-inflation-mask (round (/ padding resolution)) 1))
          (width (coerce width 'fixnum))
          (height (coerce height 'fixnum))
          (pred (if invert #'grid-value-not-occupied-p #'grid-value-occupied-p)))
      (do ((y 0 (+ y 1))) ((>= y height))
        (do ((x 0 (+ x 1))) ((>= x width))
          (when (and (funcall pred (aref map-2d y x))
                     (point-in-range-mask-p map-2d x y inflation-mask))
            (setf (aref new-map-2d y x) 127))))
      (roslisp:make-message "nav_msgs/OccupancyGrid"
                            header header
                            info info
                            data (make-array (array-dimension map-data 0) :displaced-to new-map-2d)))))

(defun map->grid-cells (map &key (invert nil))
  (roslisp:with-fields ((resolution (resolution info))
                        (height (height info))
                        (width (width info))
                        (origin-x (x position origin info))
                        (origin-y (y position origin info))
                        (map-data data))
      map
    (let ((cells nil)
          (width (round (truncate width)))
          (height (round (truncate height)))
          (occupied-pred (if invert
                             #'grid-value-not-occupied-p
                             #'grid-value-occupied-p)))
      (loop for y from 0 below height do
        (loop for x from 0 below width do
          (when (funcall occupied-pred (aref map-data (+ (* y height) x)))
            (push (make-instance 'geometry_msgs-msg:<Point>
                                 :x (+ (* x resolution) origin-x (/ resolution 2))
                                 :y (+ (* y resolution) origin-y (/ resolution 2))
                                 :z 0)
                  cells))))
      (roslisp:make-message "nav_msgs/GridCells"
                            (frame_id header) "/map"
                            (stamp header) (roslisp:ros-time)
                            cell_width resolution
                            cell_height resolution
                            cells (map 'vector #'identity cells)))))

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
                     for x-scaled = (* x resolution)
                     for y-scaled = (* y resolution)
                     when (funcall pred (aref map-data cntr)) do
                       (setf (aref cov-matrix 0 0)
                             (+ (aref cov-matrix 0 0)
                                (expt (- x-scaled origin-x (cl-transforms:x mean)) 2)))
                       (setf (aref cov-matrix 1 0)
                             (+ (aref cov-matrix 1 0)
                                (* (- x-scaled origin-x (cl-transforms:x mean))
                                   (- y-scaled origin-y (cl-transforms:y mean)))))
                       (setf (aref cov-matrix 1 1)
                             (+ (aref cov-matrix 1 1)
                                (expt (- y-scaled origin-y (cl-transforms:y mean)) 2)))
                     do (incf cntr))
            finally (progn
                      (setf (aref cov-matrix 0 0) (/ (aref cov-matrix 0 0) size))
                      (setf (aref cov-matrix 1 0) (/ (aref cov-matrix 1 0) size)) 
                      (setf (aref cov-matrix 1 1) (/ (aref cov-matrix 1 1) size))
                      (setf (aref cov-matrix 0 1) (aref cov-matrix 1 0))
                      (return cov-matrix))))))


(defun occupied-grid-metadata (map &key key)
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
