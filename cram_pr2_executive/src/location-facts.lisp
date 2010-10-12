;;;
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

(in-package :pr2-ex-reasoning)

(defun bottle-put-down-pose ()
  (tf:make-pose-stamped
   "/map" 0
   (cl-transforms:make-3d-vector -1.87 2.29 0.976)
   (cl-transforms:make-quaternion 0 0 0 1)))

(defun plate-put-down-pose ()
  (tf:make-pose-stamped
   "/map" 0
   (cl-transforms:make-3d-vector -1.75 2.10 0.976)
   (cl-transforms:make-quaternion 0 0 0 1)))


(def-fact-group pr2-specific-costmap-params (drivable-location-costmap)
  (<- (costmap-padding 0.6))
  (<- (costmap-manipulation-padding 0.4))
  (<- (costmap-in-reach-padding 1.0)))

(def-fact-group costmap-metadata (desig-loc)
  (<- (costmap-size ?width ?height)
    (symbol-value table-costmap:*map-fl* ?map-fl)
    (fluent-value ?map-fl ?map)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :width ?width)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :height ?height))

  (<- (costmap-resolution 0.05))

  (<- (costmap-origin ?x ?y)
    (symbol-value table-costmap:*map-fl* ?map-fl)
    (fluent-value ?map-fl ?map)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :origin (?x ?y)))

  (<- (desig-loc ?desig (pose ?p))
    (loc-desig? ?desig)
    (desig-prop ?desig (pr2-ex::for ?obj))
    (desig-prop ?obj (type bottle))
    (lisp-fun bottle-put-down-pose ?p)))
