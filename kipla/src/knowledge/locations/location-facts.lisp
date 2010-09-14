;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :kipla-reasoning)

;;; Currently implemented keys for location designators:
;;; (on table)
;;; (to reach) (to see)
;;; (location ?loc)
;;; (obj ?obj)
;;; (pose ?p)
;;; (of ?obj)
;;; (for ?obj-type)

(defun current-robot-axis-side (axis boundary)
  (let ((robot-pos (cl-transforms:translation
                      (cl-tf:lookup-transform
                       *tf*
                       :target-frame "/map"
                       :source-frame "/base_link"))))
    (if (< (ecase axis
             (:x (cl-transforms:x robot-pos))
             (:y (cl-transforms:y robot-pos)))
           boundary)
        :left
        :right)))

(defmethod costmap-generator-name->score ((name (eql 'x-axis-side)))
  11)

(def-fact-group rosie-specific-costmap-params (drivable-location-costmap)
    (<- (costmap-padding 0.75))
    (<- (costmap-manipulation-padding 0.6))

    (<- (drivable-location-costmap ?cm ?_)
      (costmap ?cm)
      (lisp-fun current-robot-axis-side :x -2.0 ?side)
      (costmap-add-function
       x-axis-side
       (make-axis-boundary-cost-function :x -2.0 ?side)
       ?cm)))

(def-fact-group costmap-metadata ()
  (<- (costmap-size ?width ?height)
    (symbol-value table-costmap:*map-fl* ?map-fl)
    (fluent-value ?map-fl ?map)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :width ?width)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :height ?height))

  (<- (costmap-resolution 0.05))

  (<- (costmap-origin ?x ?y)
    (symbol-value table-costmap:*map-fl* ?map-fl)
    (fluent-value ?map-fl ?map)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :origin (?x ?y))))
