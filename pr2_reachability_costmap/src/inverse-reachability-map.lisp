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

(defclass inverse-reachability-map ()
  ((origin :reader origin :initarg :origin :type cl-transforms:3d-vector)
   (size :reader size :initarg :size :type cl-transforms:3d-vector)
   (resolution :reader resolution :initarg :resolution
               :type cl-transforms:3d-vector)
   (inverse-reachability-map :reader inverse-reachability-map)))

;; Calculates for each z coordinate in `reachability-map' an inverse
;; reachability map. That is a map that contains for each coordinate
;; (x y) a value between 0 and 1 that indicates how good the point (0
;; 0 z) is reachable. A value of 1 indicates that the all orientations
;; stored in `reachability-map' are valid, a value of 0 indicates that
;; it is impossible to reach from that point.

(defun make-inverse-reachability-map (reachability-map)
  (let ((map (make-instance 'inverse-reachability-map
               :origin (minimum reachability-map)
               :size (cl-transforms:v- (maximum reachability-map)
                                       (minimum reachability-map))
               :resolution (resolution reachability-map)))
        (reachability-map-matrix (reachability-map reachability-map)))
    (with-slots (origin size resolution inverse-reachability-map) map
      (declare (type simple-array reachability-map-matrix inverse-reachability-map))
      (setf inverse-reachability-map
            (make-array (array-dimensions reachability-map-matrix) :element-type 'bit))
      (loop
        with z-size = (array-dimension inverse-reachability-map 0)
        for z from (cl-transforms:z origin) by (cl-transforms:z resolution)
        for z-index from 0 below z-size
        do (loop
             with y-size = (array-dimension inverse-reachability-map 1)
             for y from (cl-transforms:y origin) by (cl-transforms:y resolution)
             for y-index from 0 below y-size
             do (loop
                  with x-size = (array-dimension inverse-reachability-map 2)
                  for x from (cl-transforms:x origin)
                    by (cl-transforms:x resolution)
                  for x-index from 0 below x-size
                  do (copy-reachability-bitfield
                      reachability-map-matrix x-index y-index z-index
                      inverse-reachability-map
                      (- x-size x-index 1) (- y-size y-index 1) z-index)))))
    map))

(defun copy-reachability-bitfield
    (source source-x source-y source-z
     destination destination-x destination-y destination-z)
  (declare (type simple-array source destination)
           (type fixnum source-x source-y source-z)
           (type fixnum destination-x destination-y destination-z))
  (dotimes (bit (array-dimension source 3) destination)
    (setf (aref destination destination-z destination-y destination-x bit)
          (aref source source-z source-y source-x bit))))
