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

(in-package :location-costmap)

;;; This file defines the PROLOG rules that help resolution of designators based on costmaps.
;;; By itself, the file will do nothing, as it does not define any costmaps, so to use it you have to
;;; also load a file describing costmaps and providing them with prolog rules such as
;;;   (<- (desig-costmap ?desig ?cm)
;;;     (costmap ?cm)
;;;     (desig-prop ?desig (to see))
;;;     ...)
;;; examples are in table_costmap and semantic_map_costmap


(defmethod costmap-generator-name->score ((name (eql 'location-neighborhood)))
  5)

(defun make-angle-to-point-generator (pose)
  "Returns a function that takes an X and Y coordinate and returns a
quaternion to face towards `pose'"
  (lambda (x y)
    (cl-transforms:axis-angle->quaternion
     (cl-transforms:make-3d-vector 0 0 1)
     (let ((p-rel (cl-transforms:v-
                   (cl-transforms:origin pose)
                   (cl-transforms:make-3d-vector x y 0))))
       (atan (cl-transforms:y p-rel) (cl-transforms:x p-rel))))))

;; TODO: This fact belongs into a package for CRAM_PL based desig utils
(def-fact-group location-costmap-utils ()

  ;; looks up a value in a CRAM fluent
  (<- (global-fluent-value ?name ?value)
    (symbol-value ?name ?fl)
    (lisp-fun cpl:value ?fl ?value)
    (lisp-pred identity ?value)))

;; this fact group transforms messages of ROS type nav_msgs-msg:occupancygrid
;; into LISP occupancy-grid objects
(def-fact-group location-costmap ()

  (<- (occupancy-grid ?msg ?grid)
    (occupancy-grid ?msg ?grid (padding nil)))

  (<- (inverted-occupancy-grid ?msg ?grid)
    (inverted-occupancy-grid ?msg ?grid (padding nil)))

  (<- (occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)
    (lisp-type ?msg nav_msgs-msg:occupancygrid)
    (lisp-fun occupancy-grid-msg->occupancy-grid ?msg :padding ?p ?grid))

  (<- (inverted-occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)
    (lisp-type ?msg nav_msgs-msg:occupancygrid)
    (lisp-fun occupancy-grid-msg->occupancy-grid ?msg :padding ?p :invert t ?grid))

  (<- (occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)
    (lisp-type ?msg nav_msgs-msg:gridcells)
    (lisp-fun grid-cells-msg->occupancy-grid ?msg ?p ?grid))

  (<- (inverted-occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)
    (lisp-type ?msg nav_msgs-msg:gridcells)
    (lisp-fun grid-cells-msg->occupancy-grid ?msg ?p ?tmp-grid)
    (lisp-fun invert-occupancy-grid ?tmp-grid ?grid)))

;; this fact group extends location designator resolution with costmaps
(def-fact-group location-costmap-desigs (desig-costmap)

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to see))
    (costmap ?cm)
    (desig-location-prop ?desig ?loc)
    (costmap-add-function location-neighborhood (make-location-cost-function ?loc 0.5) ?cm)
    (-> (desig-location-prop ?desig ?loc)
        (costmap-add-orientation-generator (make-angle-to-point-generator ?loc) ?cm)
        (true)))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to reach))
    (desig-location-prop ?desig ?loc)
    (costmap ?cm)
    (costmap-add-function location-neighborhood (make-location-cost-function ?loc 0.4) ?cm)
    (-> (desig-location-prop ?desig ?loc)
        (costmap-add-cached-orientation-generator (make-angle-to-point-generator ?loc) ?cm)
        (true)))

  (<- (merged-desig-costmap ?desig ?cm)
    ;; bagof collects all true solutions for c into costmaps
    (bagof ?c (desig-costmap ?desig ?c) ?costmaps)
    (lisp-fun merge-costmaps ?costmaps ?cm))

  (<- (costmap-samples ?cm ?solutions)
    (lisp-fun costmap-samples ?cm ?solutions)))
