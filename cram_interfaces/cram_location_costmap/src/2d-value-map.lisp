;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(defgeneric 2d-value-map-lookup (map x y)
  (:documentation "Returns the z value of a 2D-point in the height
  map."))

(defclass 2d-value-map (occupancy-grid-metadata)
  ((2d-value-map :reader 2d-value-map)))

(defclass lazy-2d-value-map (2d-value-map)
  ((generator :initarg :generator-fun
              :initform (error
                         'simple-error
                         :format-control "No generator function specified")
              :reader generator))
  (:default-initargs :initial-element nil))

(defmethod initialize-instance :after ((map 2d-value-map) &key initial-element)
  (with-slots (width height resolution 2d-value-map) map
    (setf 2d-value-map (make-array (list (round (/ width resolution)) (round (/ height resolution)))
                                   :initial-element initial-element))))

(defmethod 2d-value-map-lookup ((map 2d-value-map) x y)
  (with-slots (resolution origin-x origin-y 2d-value-map width height) map
    (declare (type simple-array 2d-value-map))
    (if (or (< (- x origin-x) 0)
            (< (- y origin-y) 0)
            (> (- x origin-x) width)
            (> (- y origin-y) height))
        0.0d0
        (aref 2d-value-map
              (round (/ (- y origin-y) resolution))
              (round (/ (- x origin-x) resolution))))))

(declaim (inline 2d-value-map-set))
(defun 2d-value-map-set (map x y value)
  (with-slots (resolution origin-x origin-y 2d-value-map) map
    (declare (type simple-array 2d-value-map))
    (setf (aref 2d-value-map
                (round (/ (- y origin-y) resolution))
                (round (/ (- x origin-x) resolution)))
          value)))

(defmethod 2d-value-map-lookup ((map lazy-2d-value-map) x y)
  (let ((cached-value (call-next-method)))
    (or cached-value
        (2d-value-map-set map x y (funcall (generator map) x y)))))
