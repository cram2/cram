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

;;; The class location-costmap is central to this code, and the
;;; cost-functions it contains. The cost functions are used for
;;; sampling. The cost functions are combined by multiplying
;;; corresponding values for equal X- and Y-coordinates and finally
;;; normalizing the resulting map to be a valid probability
;;; distribution, i.e. to have a sum of 1. Therefore the values
;;; returned should never be negative, and at least some must be
;;; non-zero

(define-condition invalid-probability-distribution (error) ())

;; A location costmap is a 2d grid map that defines for each x,y point
;; a cost value and also a z value giving the height of the location in 3D
(defclass location-costmap (occupancy-grid-metadata)
  ((cost-map :documentation "private slot that gets filled with data by the cost-functions")
   (cost-functions :reader cost-functions :initarg :cost-functions
                   :initform nil
                   :documentation "Sequence of cons (closure . name)
                                   where the closures take an x and a
                                   y coordinate and return the
                                   corresponding cost in the interval
                                   [0;1]. name must be something for
                                   which costmap-generator-name->score
                                   is defined, and it must be unique
                                   for this costmap, as functions with
                                   the same name count as
                                   duplicates. To set this slot, use
                                   register-cost-function
                                   preferably.")
   ;; use multiple generators, e.g. first one returns a deterministic value or nil
   ;; second one a random value. Especially if we look for positions and want to
   ;; first try the current position of object or robot as solution
   (generators :accessor generators :initarg :generators
               :initform (list #'gen-costmap-sample)
               :documentation "List of generator functions that
               generate points from the costmap. If a generator
               function returns nil, the next generator function in
               the sequence is called.")
   (height-map :initform nil
               :initarg :height-map
               :reader height-map
               :documentation "An object for which method
               height-map-lookup is defined, e.g. height-map.  Use
               register-height-map to set.")))

(defgeneric get-cost-map (map)
  (:documentation "Returns the costmap as a two-dimensional array of
  FLOAT."))

(defgeneric get-map-value (map x y)
  (:documentation "Returns the cost of a specific pose."))

(defgeneric register-cost-function (map fun name)
  (:documentation "Registers a new cost function. `fun' is a function
  object that accepts two parameters, X and Y and that returns a
  number within the interval [0;1].  `name' is a unique identifier for
  which the method COSTMAP-GENERATOR-NAME->SCORE is defined and is
  used to order the different cost functions before the costmap is
  calculated."))

(defgeneric register-height-map (costmap height-map)
  (:documentation "Registers a height-map in the costmap. Throws a
  warning if the costmap already has a heightmap."))

(defgeneric generate-point (map)
  (:documentation "Generates a point and returns it."))

(defgeneric gen-costmap-sample (map)
  (:documentation "Draws a sample from the costmap `map' interpreted
  as a probability function and returns it as
  cl-transforms:3d-vector."))

(defgeneric costmap-samples (map)
  (:documentation "Returns the lazy-list of randomly generated costmap
  samples"))

(defgeneric costmap-generator-name->score (name)
  (:documentation "Returns the score for the costmap generator with
  name `name'. Greater scores result in earlier evaluation.")
  (:method ((name number))
    name))

(defmethod get-cost-map ((map location-costmap))
  "returns the cost of the location map by reading the slot, fills it if not filled yet."
  (flet ((calculate-map-value (map x y)
           (declare (type location-costmap map)
                    (type double-float x y))
           (let ((value 1.0d0))
             (dolist (cost-function (cost-functions map))
               (when (eql value 0.0d0)
                 (return-from calculate-map-value 0.0d0))
               (setq value (* value (or (funcall (car cost-function) x y)
                                        1.0d0))))
             value)))
    (declare (ftype (function (location-costmap double-float double-float) double-float)
                    calculate-map-value))
    (with-slots (width height origin-x origin-y resolution) map
      (unless (slot-boundp map 'cost-map)
        (setf (slot-value map 'cost-functions)
              (sort (remove-duplicates (slot-value map 'cost-functions)
                                       :key #'cdr)
                    #'> :key (compose #'costmap-generator-name->score #'cdr)))
        (let ((new-cost-map (cma:make-double-matrix (round (/ width resolution)) (round (/ height resolution))))
              (sum 0.0d0)
              (cost-value 0.0d0)              
              (curr-x (float origin-x 0.0d0))
              (curr-y (float origin-y 0.0d0))
              (resolution (coerce resolution 'double-float)))
          (declare (type double-float sum cost-value curr-x curr-y resolution))
          (dotimes (row (cma:height new-cost-map))
            (setf curr-x (float origin-x 0.0d0))
            (dotimes (col (cma:width new-cost-map))
              (setf cost-value (calculate-map-value map curr-x curr-y))
              (incf curr-x resolution)
              (setf sum (+ sum cost-value))
              (setf (aref new-cost-map row col) cost-value))
            (incf curr-y resolution))
          (when (= sum 0)
            (error 'invalid-probability-distribution))
          (setf (slot-value map 'cost-map) (cma:m./ new-cost-map sum))))
      (slot-value map 'cost-map))))

(defmethod get-map-value ((map location-costmap) x y)
  (aref (get-cost-map map)
        (truncate (- y (slot-value map 'origin-y))
                  (slot-value map 'resolution))
        (truncate (- x (slot-value map 'origin-x))
                  (slot-value map 'resolution))))

(defmethod register-cost-function ((map location-costmap) fun name)
  (when fun
    (push (cons fun name) (slot-value map 'cost-functions))))

(defmethod register-height-map ((map location-costmap) new-height-map)
  (with-slots (height-map) map
    (when height-map
      (warn 'simple-warning :format-control "Costmap already contains a height-map. Replacing it."))
    (setf height-map new-height-map)))

(defun merge-costmaps (cm-1 &rest costmaps)
  "merges cost functions and generators of cost maps, returns one costmap"
  (etypecase cm-1
    (list (apply #'merge-costmaps (append (force-ll cm-1) costmaps)))
    (location-costmap
       ;; Todo: assert equal size of all costmaps
       (make-instance 'location-costmap
                      :width (width cm-1)
                      :height (height cm-1)
                      :origin-x (origin-x cm-1)
                      :origin-y (origin-y cm-1)
                      :resolution (resolution cm-1)
                      :cost-functions (reduce #'append (mapcar #'cost-functions costmaps)
                                              :initial-value (cost-functions cm-1))
                      :generators (reduce #'append (mapcar #'generators costmaps)
                                          :initial-value (generators cm-1))
                      :height-map (height-map cm-1)))))

(defmethod generate-point ((map location-costmap))
  (with-slots (generators) map
    (some (rcurry #'funcall map) generators)))

(defmethod gen-costmap-sample ((map location-costmap))
  (let ((rand (random 1.0))
        (cntr 0.0)
        (cost-map (get-cost-map map)))
    (declare (type cma:double-matrix cost-map))
    (with-slots (origin-x origin-y resolution) map
      (dotimes (row (cma:height cost-map))
        (dotimes (col (cma:width cost-map))
          (setf cntr (+ cntr (aref cost-map row col)))
          (when (> cntr rand)
            (let* ((x (+ (* col resolution) origin-x))
                   (y (+ (* row resolution) origin-y))
                   (z (or (when (height-map map)
                            (height-map-lookup (height-map map) x y))
                          0.0d0)))
              (return-from gen-costmap-sample
                (cl-transforms:make-3d-vector x y z))))))))
  (error 'invalid-probability-distribution))

(defmethod costmap-samples ((map location-costmap))
  (lazy-list ()
    (cont (gen-costmap-sample map))))
