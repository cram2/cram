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
   (moveit :reader moveit :initarg :moveit :initform nil)
   (ik-link-name :reader ik-link-name :initarg :ik-link-name :initform "" :type string)
   (group-name :reader group-name :initarg :group-name :initform "" :type string)
   (ik-base-name :reader ik-base-name :initarg :ik-base-name :initform "" :type string)
   (minimum :reader minimum :initarg :minimum :type cl-transforms:3d-vector)
   (maximum :reader maximum :initarg :maximum :type cl-transforms:3d-vector)
   (resolution :reader resolution :initarg :resolution :type cl-transforms:3d-vector)
   (orientations :reader orientations :initarg :orientations :type list)
   (reachability-map :accessor reachability-map)
   (inverse-reachability-map :reader inverse-reachability-map)))

(defgeneric origin (reachability-map)
  (:method ((reachability-map reachability-map))
    (minimum reachability-map)))

(defgeneric inverse-map-origin (reachability-map)
  (:method ((reachability-map reachability-map))
    (let ((size/-2 (cl-transforms:v* (inverse-map-size reachability-map) -0.5)))
      (cl-transforms:make-3d-vector
       (cl-transforms:x size/-2)
       (cl-transforms:y size/-2)
       (- (cl-transforms:z (maximum reachability-map)))))))

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
    (with-slots (side moveit ik-link-name ik-base-name group-name) map
      (setf (slot-value map 'reachability-map)
            (if moveit
              (generate-moveit-reachability-map map ik-link-name group-name ik-base-name)
              (generate-reachability-map
                map :namespace (cdr (assoc side *arm-namespaces*))))))))

(defmethod inverse-reachability-map :before ((map reachability-map))
  (unless (slot-boundp map 'inverse-reachability-map)
    (setf (slot-value map 'inverse-reachability-map)
          (generate-inverse-reachability-map map))))

(defgeneric pose-reachability (reachability-map pose)
  (:documentation "Returns a value between 0 and 1 indicating how many
  orientations are reachable in a specific cell.")
  (:method ((reachability-map reachability-map) pose)
    (with-slots ((origin cl-transforms:origin)) pose
      (let ((matrix (reachability-map reachability-map))
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
                      (cl-transforms:z (origin reachability-map)))))
        (let ((result 0))
          (dotimes (orientation-index (length (orientations reachability-map)))
            (when (eql (aref matrix z-index y-index x-index orientation-index) 1)
              (incf result)))
          (/ result (length (orientations reachability-map))))))))

(defgeneric inverse-pose-reachability (reachability-map pose)
  (:documentation "Returns a value between 0 and 1 indicating from how
  many orientations the origin is reachable from `pose'.")
  (:method ((reachability-map reachability-map) pose)
    (with-slots ((origin cl-transforms:origin)) pose
      (let ((matrix (inverse-reachability-map reachability-map))
            (x-index (map-coordinate->array-index
                      (cl-transforms:x origin)
                      (cl-transforms:x (resolution reachability-map))
                      (cl-transforms:x (inverse-map-origin reachability-map))))
            (y-index (map-coordinate->array-index
                      (cl-transforms:y origin)
                      (cl-transforms:y (resolution reachability-map))
                      (cl-transforms:y (inverse-map-origin reachability-map))))
            (z-index (map-coordinate->array-index
                      (cl-transforms:z origin)
                      (cl-transforms:z (resolution reachability-map))
                      (cl-transforms:z (inverse-map-origin reachability-map)))))
        (let ((result 0))
          (dotimes (orientation-index (length (orientations reachability-map)))
            (when (eql (aref matrix z-index y-index x-index orientation-index) 1)
              (incf result)))
          (/ result (length (orientations reachability-map))))))))

(defgeneric pose-reachable-p (reachability-map pose &key use-closest-orientation)
  (:method ((reachability-map reachability-map) pose &key use-closest-orientation)
    (with-slots ((origin cl-transforms:origin)
                 (orientation cl-transforms:orientation))
        pose
      (let ((matrix (reachability-map reachability-map))
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
            (orientation-index
              (if use-closest-orientation
                  (nth-value
                   1 (find-closest-orientation
                      orientation (orientations reachability-map)))
                  (position
                   orientation (orientations reachability-map)
                   :test (lambda (rotation-1 rotation-2)
                           (< (cl-transforms:angle-between-quaternions
                               rotation-1 rotation-2)
                              1e-3))))))
        (when (and orientation-index
                   (> x-index 0) (> y-index 0) (> z-index 0)
                   (< x-index (array-dimension matrix 2))
                   (< y-index (array-dimension matrix 1))
                   (< z-index (array-dimension matrix 0)))
          (eql (aref matrix z-index y-index x-index orientation-index) 1))))))

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
                                               ;(format t ".")
                                               (setf (aref map z-index y-index x-index angle-index) 1))
                                              (t ;(format t "x")
                                                 (setf (aref map z-index y-index x-index angle-index) 0))))
                            finally (format t "~%")))
          finally (return map))))

(defun generate-moveit-reachability-map (reachability-map link-name group-name ik-base-name)
  (with-slots (minimum maximum resolution orientations) reachability-map
    (setf (reachability-map reachability-map)
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
                                       do (cond ((moveit-find-ik-solution
                                                   :pose (cl-transforms:make-pose
                                                           (cl-transforms:make-3d-vector x y z)
                                                           angle)
                                                   :ik-link-name link-name
                                                   :group-name group-name
                                                   :ik-base-frame-name ik-base-name)
                                                 ;;(format t ".")
                                                 (setf (aref map z-index y-index x-index angle-index) 1))
                                                (t ;;(format t "x")
                                                   (setf (aref map z-index y-index x-index angle-index) 0))))
                              finally (format t "~%")))
            finally (return map)))
    reachability-map))

(defun generate-inverse-reachability-map (reachability-map)
  "Returns a new four-dimensional array that represents an inverse
reachability map. The map is generated from `reachability-map'. Each
entry indicates from where the ((0 0 0) `orientation') for all
orientations of `reachability-map' is reachable. The format of the
result is: (z y x orientation)"
  (flet ((origin-reachable-p (robot-point goal-orientation)
           (some (lambda (orientation)
                   (let ((pose (cl-transforms:transform-pose
                                (cl-transforms:transform-inv
                                 (cl-transforms:make-transform
                                  robot-point orientation))
                                (cl-transforms:make-pose
                                 (cl-transforms:make-identity-vector)
                                 goal-orientation))))
                     (pose-reachable-p
                      reachability-map pose :use-closest-orientation t)))
                 (orientations reachability-map))))
    (let* ((reachability-map-matrix (reachability-map reachability-map))
           (inverse-map-size (max (array-dimension reachability-map-matrix 2)
                                  (array-dimension reachability-map-matrix 1)))
           (inverse-reachability-map-matrix
             (make-array
              (list (array-dimension reachability-map-matrix 0)
                    inverse-map-size inverse-map-size
                    (array-dimension reachability-map-matrix 3))
              :element-type 'bit)))
      (loop for z-index from 0 below (array-dimension reachability-map-matrix 0)
            for z from (- (cl-transforms:z (maximum reachability-map)))
              by (cl-transforms:z (resolution reachability-map))
            do (format t "z: ~a~%" z)
               (loop for y-index from 0 below inverse-map-size
                     for y from (* (truncate inverse-map-size -2)
                                   (cl-transforms:y (resolution reachability-map)))
                       by (cl-transforms:y (resolution reachability-map))
                     do (format t "y: ~a~%" y)
                        (loop for x-index from 0 below inverse-map-size
                              for x from (* (truncate inverse-map-size -2)
                                            (cl-transforms:x (resolution reachability-map)))
                                by (cl-transforms:x (resolution reachability-map))
                              do (loop for orientation in (orientations reachability-map)
                                       for orientation-index from 0 do
                                         (setf (aref inverse-reachability-map-matrix
                                                     z-index y-index x-index
                                                     orientation-index)
                                               (cond ((origin-reachable-p
                                                       (cl-transforms:make-3d-vector x y z) orientation)
                                                       ;;(format t ".")
                                                       1)
                                                     (t 
                                                       ;;(format t "x")
                                                       0)))))
                        (format t "~%")))
      inverse-reachability-map-matrix)))

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

(defun find-closest-orientation (reference-orientation orientations)
  (let ((current-best-index 0)
        (current-best-angle (abs (cl-transforms:angle-between-quaternions
                                  reference-orientation (car orientations))))
        (current-best-orientation (car orientations)))
    (loop
      for index from current-best-index
      for orientation in (cdr orientations)
      for angle = (abs (cl-transforms:angle-between-quaternions
                        reference-orientation orientation))
      when (< angle current-best-angle) do
        (setf current-best-index index)
        (setf current-best-angle angle)
        (setf current-best-orientation orientation)
      finally (return (values current-best-orientation current-best-index)))))
