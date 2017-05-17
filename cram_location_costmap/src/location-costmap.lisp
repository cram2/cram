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

;;; The class location-costmap is central to this code, and the
;;; cost-functions it contains. The cost functions are used for
;;; sampling. The cost functions are combined by multiplying
;;; corresponding values for equal X- and Y-coordinates and finally
;;; normalizing the resulting map to be a valid probability
;;; distribution, i.e. to have a sum of 1. Therefore the values
;;; returned should never be negative, and at least some must be
;;; non-zero

(define-condition no-cost-functions-registered (error) ())

;; A location costmap is a 2d grid map that defines for each x,y point
;; a cost value. It provides an API to sample random poses by
;; generating random x and y points and using further generators to
;; set the z value and the orientation of the pose.
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
   (height-generators
    :initform nil :initarg :height-generators :reader height-generators
    :documentation "A list of callable objects that take two
                    parameters, X and Y and return a list of valid
                    height values (i.e. Z coordinate of the generated
                    pose). If no height generator is defined, a constant
                    value of 0.0d0 is returned.")
   (orientation-generators
    :initform nil :initarg :orientation-generators :reader orientation-generators
    :documentation "A sequence of callable objects that take three
                    parameters, X, Y and the return value of the
                    previous orientation generators. Each of the
                    functions returns a (lazy-) list of
                    quaternions. The function is not required to be
                    deterministic and called whenever a new sample is
                    generated.")))

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

(defgeneric register-height-generator (costmap generator)
  (:documentation "Registers a height generator in the costmap"))

(defgeneric register-orientation-generator (costmap generator)
  (:documentation "Registers an orientation generator in the
  costmap"))

(defgeneric gen-costmap-sample-point (map &key sampling-function)
  (:documentation "Draws a sample from the costmap `map' interpreted
  as a probability function and returns it as CL-TRANSFORMS:3D-VECTOR"))

(defgeneric costmap-samples (map &key sampling-function)
  (:documentation "Returns the lazy-list of randomly generated costmap
  samples, i.e. a lazy-list of instances of type CL-TRANSFORMS:POSE"))

(defgeneric costmap-generator-name->score (name)
  (:documentation "Returns the score for the costmap generator with
  name `name'. Greater scores result in earlier evaluation.")
  (:method ((name number))
    name))

(define-hook on-visualize-costmap (costmap))
(define-hook on-visualize-costmap-sample (point))

(defmethod get-cost-map ((map location-costmap))
  "Returns the costmap matrix of `map', i.e. if not generated yet,
calls the generator functions and runs normalization."
  (unless (cost-functions map)
    (error 'no-cost-functions-registered))
  (with-slots (width height origin-x origin-y resolution) map
    (unless (slot-boundp map 'cost-map)
      (setf (slot-value map 'cost-functions)
            (sort (remove-duplicates (slot-value map 'cost-functions)
                                     :key #'generator-name)
                  #'> :key (compose
                            #'costmap-generator-name->score
                            #'generator-name)))
      (let ((new-cost-map (cma:make-double-matrix
                           (truncate (/ width resolution))
                           (truncate (/ height resolution))
                           :initial-element 1.0d0))
            (sum 0.0d0))
        (dolist (generator (cost-functions map))
          (setf new-cost-map (generate generator new-cost-map map)))
        (dotimes (row (cma:height new-cost-map))
          (dotimes (column (cma:width new-cost-map))
            (incf sum (aref new-cost-map row column))))
        (when (= sum 0)
          (error 'invalid-probability-distribution))
        (setf (slot-value map 'cost-map) (cma:m./ new-cost-map sum))
        (on-visualize-costmap map)))
    (slot-value map 'cost-map)))

(defmethod get-map-value ((map location-costmap) x y)
  (aref (get-cost-map map)
        (truncate (- y (slot-value map 'origin-y))
                  (slot-value map 'resolution))
        (truncate (- x (slot-value map 'origin-x))
                  (slot-value map 'resolution))))

(defmethod register-cost-function ((map location-costmap) (function function) name)
  (push (make-instance 'function-costmap-generator
          :name name :generator-function function)
        (slot-value map 'cost-functions)))

(defmethod register-cost-function
    ((map location-costmap) (generator costmap-generator) name)
  (setf (slot-value generator 'name) name)
  (push generator (slot-value map 'cost-functions)))

(defmethod register-height-generator ((map location-costmap) generator)
  (with-slots (height-generators) map
    (setf height-generators (append height-generators (list generator)))))

(defmethod register-orientation-generator ((map location-costmap) generator)
  (with-slots (orientation-generators) map
    (pushnew generator orientation-generators)))

(defun merge-costmaps (cm-1 &rest costmaps)
  "merges cost functions and copies one height-map, returns one costmap"
  (etypecase cm-1
    (list
     (apply #'merge-costmaps (append (force-ll cm-1) costmaps)))
    (location-costmap
     ;; Todo: assert equal size of all costmaps
     (make-instance 'location-costmap
       :width (width cm-1)
       :height (height cm-1)
       :origin-x (origin-x cm-1)
       :origin-y (origin-y cm-1)
       :resolution (resolution cm-1)
       ;:visualization-z (visualization-z cm-1)
       :cost-functions (reduce #'append (mapcar #'cost-functions costmaps)
                               :initial-value (cost-functions cm-1))
       :height-generators (mapcan (compose #'copy-list #'height-generators)
                                  (cons cm-1 costmaps))
       :orientation-generators (mapcan (compose #'copy-list #'orientation-generators)
                                       (cons cm-1 costmaps))))))

(defun generate-height (map x y &optional (default 0.0d0))
  (or
   (when (height-generators map)
     (let ((heights (generate-heights map x y)))
       (when heights
         (random-elt heights))))
   default))

(defun generate-heights (map x y)
  (when (height-generators map)
    (mapcan (lambda (generator)
              (copy-list (funcall generator x y)))
            (height-generators map))))

(defun generate-orientations (map x y
                              &optional (default (list (cl-transforms:make-identity-rotation))))
  (or (when (orientation-generators map)
        (reduce (lambda (solution function)
                  (funcall function x y solution))
                (orientation-generators map) :initial-value nil))
      default))

(defmethod gen-costmap-sample-point ((map location-costmap)
                                     &key (sampling-function #'cma:sample-from-distribution-matrix))
  (let ((cost-map (get-cost-map map)))
    (declare (type cma:double-matrix cost-map))
    (destructuring-bind (column row)
        (funcall sampling-function cost-map)
      (with-slots (origin-x origin-y resolution) map
        (let* ((x (+ (* column resolution) origin-x))
               (y (+ (* row resolution) origin-y))
               (z (generate-height map x y))
               (point (cl-transforms:make-3d-vector x y z)))
          (on-visualize-costmap-sample point)
          (on-visualize-costmap map)
          point)))))

(defmethod costmap-samples ((map location-costmap) &key sampling-function)
  (lazy-mapcan (lambda (sample-point)
                 (lazy-mapcar (lambda (orientation)
                                (cl-transforms:make-pose sample-point orientation))
                              (generate-orientations
                               map
                               (cl-transforms:x sample-point)
                               (cl-transforms:y sample-point))))
               (lazy-list ()
                 (cont (if sampling-function
                           (gen-costmap-sample-point
                            map :sampling-function sampling-function)
                           (gen-costmap-sample-point map))))))
