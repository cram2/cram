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

(defun make-padding-mask (size)
  "returns a 2d square array with an odd dimension drawing a discretized circle
of radius `size' in grid space"
  (unless (<= size 0)
    (let* ((array-width (1+ (* size 2)))
           (mask (make-array (list array-width array-width) :element-type 'fixnum))
           (dist^2 (expt size 2)))
      (declare (type (simple-array fixnum *) mask))
      (dotimes (row array-width)
        (dotimes (col array-width)
          ;; (size, size) is the center cell
          (if (<= (+ (expt (- col size) 2) (expt (- row size) 2))
                  dist^2)
              (setf (aref mask row col) 1)
              (setf (aref mask row col) 0))))
      mask)))

(declaim (ftype (function ((simple-array fixnum 2) fixnum fixnum (simple-array fixnum 2))
                          boolean) point-in-range-mask-p))
(defun point-in-padding-mask-p (map x y range-mask)
  (declare (type (simple-array fixnum 2) map range-mask)
           (type fixnum x y))
  (let ((array-height (array-dimension map 0))
        (array-width (array-dimension map 1))
        (mask-height (array-dimension range-mask 0))
        (mask-width (array-dimension range-mask 1)))
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
              (return-from point-in-padding-mask-p t)))))))
  nil)

(defun occupancy-grid-put-mask (x y grid mask &key (coords-raw-p nil))
  "Puts the mask into grid, at position x and y. Please note that the
  mask must completely fit, i.e. x/resolution must be >= 0.5
  mask-size-x. `coords-raw-p indicates if x and y are direct indices
  in the grid array or floating point coordinates in the reference
  coordinate system."
  (let ((x (if coords-raw-p x (round (/ (- x (origin-x grid)) (resolution grid)))))
        (y (if coords-raw-p y (round (/ (- y (origin-y grid)) (resolution grid)))))
        (grid-arr (grid grid))
        (mask-size-x (array-dimension mask 1))
        (mask-size-y (array-dimension mask 0)))
    (declare (type fixnum x y)
             (type (simple-array fixnum *) grid-arr mask))
    (do ((grid-row (- y (- (truncate (/ (array-dimension mask 0) 2)) 1))
           (+ grid-row 1))
         (mask-row 0 (+ mask-row 1)))
        ((>= mask-row (- mask-size-y 1)))
      (do ((grid-col (- x (- (truncate (/ (array-dimension mask 1) 2)) 1))
             (+ grid-col 1))
           (mask-col 0 (+ mask-col 1)))
          ((>= mask-col (- mask-size-x 1)))
        (when (eql (aref mask mask-row mask-col) 1)
          (setf (aref grid-arr grid-row grid-col) 1))))))
