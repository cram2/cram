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

(deftype occupancy-grid-data () '(simple-array fixnum 2))

(defclass occupancy-grid-metadata ()
  ((width :reader width :initarg :width :type double-float
          :reader grid-width)
   (height :reader height :initarg :height :type double-float
           :reader grid-height)
   (origin-x :reader origin-x :initarg :origin-x :type double-float)
   (origin-y :reader origin-y :initarg :origin-y :type double-float)
   (resolution :reader resolution :initarg :resolution :type double-float)
   ;; (visualization-z :reader visualization-z :initarg :visualization-z :type double-float)
   ))

(defclass occupancy-grid (occupancy-grid-metadata)
  ((grid :reader grid :type (simple-array fixnum 2)))
  (:documentation "Defines an occupancy grid. Please note that in
  contrast to ros' OccupancyGrid message, this grid only contains
  values of 0 or 1."))

(defmethod initialize-instance :after ((grid occupancy-grid) &key)
  (with-slots (width height resolution grid) grid
    (setf grid (make-array (list (round (/ height resolution))
                                 (round (/ width resolution)))
                           :element-type 'fixnum
                           :initial-element 0))))

(defun make-occupancy-grid (width height origin-x origin-y resolution &optional initial-data)
  (check-type initial-data (or null (array * *)))
  (let ((grid (make-instance 'occupancy-grid
                             :width width :height height
                             :origin-x origin-x :origin-y origin-y
                             :resolution resolution)))
    (when initial-data
      (map-into (make-array (array-total-size (grid grid))
                            :element-type 'fixnum :displaced-to (grid grid))
                #'identity (make-array (array-total-size initial-data)
                                       :element-type (array-element-type initial-data)
                                       :displaced-to initial-data)))
    grid))

(defun copy-occupancy-grid (src)
  (make-occupancy-grid (width src) (height src)
                       (origin-x src) (origin-y src)
                       (resolution src) (grid src)))

(defun invert-occupancy-grid (src)
  (let* ((result (make-occupancy-grid (width src) (height src) (origin-x src) (origin-y src)
                                      (resolution src)))
         (src-grid (grid src))
         (result-grid (grid result)))
    (declare (type occupancy-grid-data src-grid result-grid))
    (dotimes (row (array-dimension src-grid 0))
      (dotimes (col (array-dimension src-grid 1))
        (setf (aref result-grid row col)
              (if (eql (aref src-grid row col) 0) 1 0))))
    result))

(defun set-grid-cell (grid x y)
  (let ((grid-arr (grid grid)))
    (declare (type occupancy-grid-data grid-arr))
    (setf (aref grid-arr
                (round (/ (- y (origin-y grid)) (resolution grid)))
                (round (/ (- x (origin-x grid)) (resolution grid))))
          1)))

(defun clear-grid-cell (grid x y)
  (let ((grid-arr (grid grid)))
    (declare (type occupancy-grid-data grid-arr))
    (setf (aref grid-arr
                (round (/ (- y (origin-y grid)) (resolution grid)))
                (round (/ (- x (origin-x grid)) (resolution grid))))
          0)))

(defun get-grid-cell (grid x y)
  (let ((grid-arr (grid grid)))
    (declare (type occupancy-grid-data grid-arr))
    (aref grid-arr
          (round (/ (- y (origin-y grid)) (resolution grid)))
          (round (/ (- x (origin-x grid)) (resolution grid))))))

(defun matrix->occupancy-grid (origin-x origin-y resolution matrix &optional (threshold 0.0d0))
  "Creates an occupancy grid from the matrix. Sets all values above
  `threshold' to 1."
  (let* ((grid (make-occupancy-grid (* (array-dimension matrix 1) resolution)
                                   (* (array-dimension matrix 0) resolution)
                                   origin-x origin-y
                                   resolution))
         (grid-arr (grid grid)))
    (declare (type (simple-array * 2) matrix))
    (dotimes (y (array-dimension matrix 0))
      (dotimes (x (array-dimension matrix 1))
        (when (> (aref matrix y x) threshold)
          (setf (aref grid-arr y x) 1))))
    grid))
