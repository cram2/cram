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

(defclass location-costmap (occupancy-grid-metadata)
  ((cost-map)
   (cost-functions :reader cost-functions :initarg :cost-functions
                   :initform nil
                   :documentation "Sequence of closures that take an a
                                   x and a y coordinate and return the
                                   corresponding cost in the interval
                                   [0;1]")
   (generators :reader generators :initarg :generators
               :initform (list #'gen-costmap-sample)
               :documentation "List of generator functions that
               generate points from the costmap. If a generator
               function returns nil, the next generator function in
               the sequece is called.")))

(defgeneric get-cost-map (map)
  (:documentation "Returns the costmap as a two-dimensional array of
  FLOAT."))

(defgeneric get-map-value (map x y)
  (:documentation "Returns the cost of a specific pose."))

(defgeneric register-cost-function (map fun &optional score)
  (:documentation "Registers a new cost function. The optional
  argument `score' allows to define an order in which to apply each
  cost function. Higher scores are evaluated first."))

(defgeneric generate-point (map)
  (:documentation "Generates a point and returns it."))

(defgeneric gen-costmap-sample (map)
  (:documentation "Draws a sample from the costmap `map' interpreted
  as a probability function and returns it as
  cl-transforms:3d-vector."))

(defmethod get-cost-map ((map location-costmap))
  (flet ((calculate-map-value (map x y)
           (declare (type location-costmap map)
                    (type double-float x y))
           (let ((value 1.0d0))
             (dolist (cost-function (cost-functions map))
               (when (eql value 0.0d0)
                 (return-from calculate-map-value 0.0d0))
               (setq value (* value (funcall (car cost-function) x y))))
             value)))
    (declare (ftype (function (location-costmap double-float double-float) double-float)
                    calculate-map-value))
    (with-slots (width height origin-x origin-y resolution) map
      (check-type width integer)
      (check-type height integer)
      (unless (slot-boundp map 'cost-map)
        (setf (slot-value map 'cost-functions)
              (sort (slot-value map 'cost-functions) #'> :key #'cdr))
        (let ((new-cost-map (cma:make-double-matrix (round (/ width resolution)) (round (/ height resolution))))
              (sum 0.0d0)
              (cost-value 0.0d0)              
              (curr-x origin-x)
              (curr-y origin-y)
              (resolution (coerce resolution 'double-float)))
          (declare (type double-float sum cost-value curr-x curr-y resolution))
          (dotimes (row (cma:height new-cost-map))
            (setf curr-x origin-x)
            (dotimes (col (cma:width new-cost-map))
              (setf cost-value (calculate-map-value map curr-x curr-y))
              (incf curr-x resolution)
              (setf sum (+ sum cost-value))
              (setf (aref new-cost-map row col) cost-value))
            (incf curr-y resolution))
          (cma:m./ new-cost-map sum)
          (setf (slot-value map 'cost-map) new-cost-map)))
      (slot-value map 'cost-map))))

(defmethod get-map-value ((map location-costmap) x y)
  (aref (get-cost-map map)
        (truncate (- x (slot-value map 'origin-x))
                  (slot-value map 'resolution))
        (truncate (- y (slot-value map 'origin-y))
                  (slot-value map 'resolution))))

(defmethod register-cost-function ((map location-costmap) fun &optional (score 0))
  (push (cons fun score) (slot-value map 'cost-functions)))

(defun merge-costmaps (cm-1 &rest costmaps)
  (etypecase cm-1
    (list (apply #'merge-costmaps (append cm-1 costmaps)))
    (location-costmap
       ;; Todo: assert euqal size of all costmaps
       (make-instance 'location-costmap
                      :width (width cm-1)
                      :height (height cm-1)
                      :origin-x (origin-x cm-1)
                      :origin-y (origin-y cm-1)
                      :resolution (resolution cm-1)
                      :cost-functions (reduce #'append (mapcar #'cost-functions costmaps)
                                              :initial-value (cost-functions cm-1))))))

(defmethod generate-point ((map location-costmap))
  (with-slots (generators) map
    (some (rcurry #'funcall map) generators)))

(defmethod gen-costmap-sample ((map location-costmap))
  (flet ((make-var-fun ()
           "Returns a function that maps a number within the interval
            [0;1) to a coordinate that can be used in the probability
            function."
           (with-slots (width height origin-x origin-y resolution) map
             (let ((n-pts (* width height (/ resolution))))
               (lambda (v)
                 (let ((index (* v n-pts)))
                   (cons (+ (mod index height) origin-x)
                         (+ (/ index (/ height resolution)) origin-y)))))))
         (p-fun (pt)
           (get-map-value map (cdr pt) (car pt))))
    (with-slots (origin-x origin-y) map
      (let ((coord (cma:sample (make-var-fun) #'p-fun)))
        (cl-transforms:make-3d-vector (car coord) (cdr coord) 0)))))
