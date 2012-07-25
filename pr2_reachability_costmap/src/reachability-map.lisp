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

(in-package :pr2-reachability-costmap)

(defparameter *arm-namespaces* `((:left . "/reasoning/pr2_left_arm_kinematics")
                                 (:right . "/reasoning/pr2_right_arm_kinematics")))

(defclass reachability-map ()
  ((side :reader side :initarg :side :type (or :left :right))
   (minimum :reader minimum :initarg :minimum :type cl-transforms:3d-vector)
   (maximum :reader maximum :initarg :maximum :type cl-transforms:3d-vector)
   (resolution :reader resolution :initarg :resolution ::type cl-transforms:3d-vector)
   (orientations :reader orientations :initarg :orientations :type list)
   (reachability-map :reader reachability-map)
   (inverse-reachability-map :reader inverse-reachability-map)))

(defgeneric origin (reachability-map)
  (:method ((reachability-map reachability-map))
    (minimum reachability-map)))

(defgeneric inverse-map-origin (reachability-map)
  (:method ((reachability-map reachability-map))
    (let* ((reachability-map-matrix (reachability-map reachability-map))
           (inverse-map-matrix (inverse-reachability-map reachability-map))
           (resolution (resolution reachability-map))
           (size-difference-x (* (- (array-dimension reachability-map-matrix 2)
                                    (array-dimension inverse-map-matrix 2))
                                 (cl-transforms:x resolution)))
           (size-difference-y (* (- (array-dimension reachability-map-matrix 1)
                                    (array-dimension inverse-map-matrix 1))
                                 (cl-transforms:y resolution))))
      (cl-transforms:v+ (origin reachability-map)
                        (cl-transforms:v*
                         (cl-transforms:make-3d-vector
                          size-difference-x size-difference-y 0)
                         0.5)))))

(defgeneric size (reachability-map)
  (:method ((reachability-map reachability-map))
    (cl-transforms:v-
     (maximum reachability-map)
     (minimum reachability-map))))

(defgeneric inverse-map-size (reachability-map)
  (:method ((reachability-map reachability-map))
    (with-slots (resolution) reachability-map
      (let ((matrix (inverse-reachability-map reachability-map)))
        (cl-transforms:make-3d-vector
         (array-index->map-coordinate
          (1- (array-dimension matrix 2)) (cl-transforms:x resolution) 0)
         (array-index->map-coordinate
          (1- (array-dimension matrix 1)) (cl-transforms:y resolution) 0)
         (array-index->map-coordinate
          (1- (array-dimension matrix 0)) (cl-transforms:z resolution) 0))))))

(defmethod initialize-instance :after ((map reachability-map) &key filename)
  (when filename
    (restore-reachability-map map filename)))

(defmethod reachability-map :before ((map reachability-map))
  (unless (slot-boundp map 'reachability-map)
    (with-slots (side) map
      (setf (slot-value map 'reachability-map)
            (generate-reachability-map
             map :namespace (cdr (assoc side *arm-namespaces*)))))))

(defmethod inverse-reachability-map :before ((map reachability-map))
  (unless (slot-boundp map 'inverse-reachability-map)
    (setf (slot-value map 'inverse-reachability-map)
          (generate-inverse-reachability-map map))))

(defgeneric pose-reachable-p (reachability-map pose)
  (:method ((reachability-map reachability-map) pose)
    (with-slots ((origin cl-transforms:origin)
                 (orientation cl-transforms:orientation))
        pose
      (let ((reachability-map (reachability-map reachability-map))
            (x-index (map-coordinate->array-index
                      (cl-transforms:x origin)
                      (cl-transforms:x (resolution reachability-map))
                      (cl-transforms:x (origin reachability-map))))
            (y-index (map-coordinate->array-index
                      (cl-transforms:y origin)
                      (cl-transforms:y (resolution reachability-map))
                      (cl-transforms:y (origin reachability-map))))
            (z-index (map-coordinate->array-index
                      (cl-transforms:z origin)
                      (cl-transforms:z (resolution reachability-map))
                      (cl-transforms:z (origin reachability-map))))
            (orientation-index (position
                                orientation (orientations reachability-map)
                                :test (lambda (rotation-1 rotation-2)
                                        (< (cl-transforms:angle-between-quaternions
                                            rotation-1 rotation-2)
                                           1e-6)))))
        (when (and orientation-index
                   (> x-index 0) (> y-index 0) (> z-index 0)
                   (< x-index (array-dimension reachability-map 2))
                   (< y-index (array-dimension reachability-map 1))
                   (< z-index (array-dimension reachability-map 0)))
          (eql (aref reachability-map z-index y-index x-index orientation-index)
               1))))))

(defun generate-reachability-map (map &key namespace)
  (with-slots (minimum maximum resolution orientations) map
    (loop with map = (make-array
                      (list
                       (1+ (round (- (cl-transforms:z maximum)
                                     (cl-transforms:z minimum))
                                  (cl-transforms:z resolution)))
                       (1+ (round (- (cl-transforms:y maximum)
                                     (cl-transforms:y minimum))
                                  (cl-transforms:y resolution)))
                       (1+ (round (- (cl-transforms:x maximum)
                                     (cl-transforms:x minimum))
                                  (cl-transforms:x resolution)))
                       (length orientations))
                      :element-type 'bit)
          for z from (cl-transforms:z minimum)
            by (cl-transforms:z resolution)
          for z-index from 0 below (array-dimension map 0)
          do (loop for y from (cl-transforms:y minimum)
                     by (cl-transforms:y resolution)
                   for y-index from 0 below (array-dimension map 1)
                   do (format t "y: ~a z: ~a~%" y z)
                      (loop for x from (cl-transforms:x minimum)
                              by (cl-transforms:x resolution)
                            for x-index from 0 below (array-dimension map 2)
                            do (loop for angle in orientations
                                     for angle-index from 0
                                     do (cond ((find-ik-solution
                                                :pose (cl-transforms:make-pose
                                                       (cl-transforms:make-3d-vector x y z)
                                                       angle)
                                                :service-namespace namespace)
                                               (format t ".")
                                               (setf (aref map z-index y-index x-index angle-index) 1))
                                              (t (format t "x")
                                                 (setf (aref map z-index y-index x-index angle-index) 0))))
                            finally (format t "~%")))
          finally (return map))))

(defun generate-inverse-reachability-map (reachability-map)
  "Returns a new four-dimensional array that represents an inverse
reachability map. The map is generated from `reachability-map'. Each
entry indicates from where the ((0 0 0) `orientation') for all
orientations of `reachability-map' is reachable. The format of the
result is: (z y x orientation)"
  (flet ((get-reachability-map-x-y (base-pose width height orientation)
           (let* ((center (cl-transforms:make-3d-vector
                           (/ width 2) (/ height 2) 0))
                  (transformed-base-pose
                    (tf:v+ (cl-transforms:rotate
                            orientation (cl-transforms:v-inv
                                         (cl-transforms:v-
                                          base-pose center)))
                           center))
                  (rounded-x (round (cl-transforms:x transformed-base-pose)))
                  (rounded-y (round (cl-transforms:y transformed-base-pose))))
             (if (or (not (= (cl-transforms:z transformed-base-pose) 0))
                     (< rounded-x 0) (>= rounded-x width)
                     (< rounded-y 0) (>= rounded-y height))
                 (values nil nil)
                 (values rounded-x rounded-y)))))
    (let* ((reachability-map-matrix (reachability-map reachability-map))
           (orientations (orientations reachability-map))
           (inverse-map-size (max (array-dimension reachability-map-matrix 2)
                                  (array-dimension reachability-map-matrix 1)))
           (inverse-reachability-map-matrix
             (make-array
              (list (array-dimension reachability-map-matrix 0)
                    inverse-map-size inverse-map-size
                    (array-dimension reachability-map-matrix 3))
              :element-type 'bit))
           (width (array-dimension reachability-map-matrix 2))
           (height (array-dimension reachability-map-matrix 1))
           (x-offset (/ (- inverse-map-size width) 2))
           (y-offset (/ (- inverse-map-size height) 2)))
      (dotimes (z (array-dimension reachability-map-matrix 0) inverse-reachability-map-matrix)
        (dotimes (y inverse-map-size)
          (dotimes (x inverse-map-size)
            (loop for goal-orientation in orientations
                  for goal-orientation-index from 0 do
                    (loop for current-orientation in orientations
                          for i from 0
                          with inverse-orientation = (cl-transforms:q-inv goal-orientation)
                          with base-pose = (cl-transforms:make-3d-vector
                                            (- x x-offset) (- y y-offset) 0)
                          do (multiple-value-bind (x-in-base y-in-base)
                                 (get-reachability-map-x-y
                                  base-pose width height
                                  (cl-transforms:q* inverse-orientation current-orientation))
                               (when (and x-in-base y-in-base
                                          (eql (aref reachability-map-matrix
                                                     z y-in-base x-in-base i)
                                               1))
                                 (setf (aref inverse-reachability-map-matrix
                                             z y x goal-orientation-index)
                                       1)
                                 (return)))))))))))

(defun store-reachability-map (map filename)
  (let ((reachability-map (reachability-map map))
        (inverse-reachability-map (inverse-reachability-map map)))
    (with-slots (side minimum maximum resolution orientations) map
      (with-open-file (file filename
                            :direction :output
                            :element-type 'unsigned-byte
                            :if-exists :supersede)
        (store file side)
        (store file minimum)
        (store file maximum)
        (store file resolution)
        (store file orientations)
        (store file reachability-map)
        (store file inverse-reachability-map)))))

(defun restore-reachability-map (map filename)
  (with-slots (side minimum maximum resolution orientations
               reachability-map inverse-reachability-map)
      map
    (with-open-file (file filename :direction :input :element-type 'unsigned-byte)
      (setf side (restore file 'symbol))
      (setf minimum (restore file 'cl-transforms:3d-vector))
      (setf maximum (restore file 'cl-transforms:3d-vector))
      (setf resolution (restore file 'cl-transforms:3d-vector))
      (setf orientations (restore file 'list))
      (setf reachability-map (restore file 'array))
      (setf inverse-reachability-map (restore file 'array)))))
