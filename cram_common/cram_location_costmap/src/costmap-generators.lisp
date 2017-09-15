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

(in-package :location-costmap)

(defclass costmap-generator ()
  ((name :initarg :name :reader generator-name)
   (generator-function
    :initarg :generator-function :reader generator-function))
  (:documentation "Base class for all costmap generators."))

(defclass map-costmap-generator (costmap-generator) ()
  (:documentation "Generator based on a generator function that takes
  a CMA:DOUBLE-MATRIX as input, adds its costmap values and returns the
  resulting matrix. The generator function is allowed to change the
  input map. The signature of the generator function is:

    lambda (occupancy-grid-metadata matrix)"))

(defclass function-costmap-generator (costmap-generator) ()
  (:documentation "Generator based on a generator function that takes
  two parameters, the x and y coordinate and returns the corresponding
  value. The signature of the generator is:

    lambda (x y)"))

(defgeneric generate (generator map occupancy-grid-metadata)
  (:documentation "Executes `generator' on the CMA:DOUBLE-MATRIX `map'
  that has the properties specified in `costmap-metadata' and returns
  the updated `map'. This function is allowed but not required to
  change `map'."))

(defmethod generate ((generator map-costmap-generator) map
                     (costmap-metadata occupancy-grid-metadata))
  (declare (type cma:double-matrix map))
  (with-slots (generator-function) generator
    (with-slots (origin-x origin-y width height resolution) costmap-metadata
      (let ((result (funcall generator-function costmap-metadata
                             (cma:make-double-matrix
                              (truncate width resolution)
                              (truncate height resolution)))))
        (declare (type cma:double-matrix result))
        (cma:m* map result)))))

(defmethod generate ((generator function-costmap-generator) map
                     (costmap-metadata occupancy-grid-metadata))
  (declare (type cma:double-matrix map))
  (with-slots (origin-x origin-y width height resolution) costmap-metadata
    (loop for y-index from 0 below (truncate height resolution)
          for y from origin-y by resolution do
            (loop for x-index from 0 below (truncate width resolution)
                  for x from origin-x by resolution do
                    (unless (eql (aref map y-index x-index) 0.0d0)
                      (setf (aref map y-index x-index)
                            (* (aref map y-index x-index)
                               (funcall (generator-function generator) x y)))))
          finally (return map))))

(defun map-coordinate->array-index (coordinate resolution origin)
  "Given a coordinate value, the resolution of the map and the origin
of the coordinate's axis calculates the corresponding index in an
array."
  (declare (type number coordinate resolution origin))
  (truncate (- coordinate origin) resolution))

(defun array-index->map-coordinate (index resolution origin)
  "Given an array index a map resolution and the origin of the axis
index is in, returns the corresponding coordinate value."
  (declare (type fixnum index)
           (type number resolution origin))
  (+ (* index resolution) origin))
