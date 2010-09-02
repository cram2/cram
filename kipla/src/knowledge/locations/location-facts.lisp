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

(in-package :kipla-reasoning)

;;; Currently implemented keys for location designators:
;;; (on table)
;;; (to reach) (to see)
;;; (location ?loc)
;;; (obj ?obj)
;;; (pose ?p)
;;; (of ?obj)
;;; (for ?obj-type)

(defun obj-desig-location (obj-desig)
  (when (and (typep obj-desig 'object-designator)
             (desig-prop-value obj-desig 'at))
    (reference (current-desig (desig-prop-value obj-desig 'at)))))

(defun loc-desig-location (loc-desig)
  (when (and (typep loc-desig 'location-designator)
             loc-desig)
    (reference loc-desig)))

(defun make-robot-pos-generator (threshold &key (n-solutions 1))
  "Returns a function that returns the position of the robot in the
map if its probability value in the corresponding costmap is greater
than threshold * highest-probability."
  (flet ((max-map-value (map-arr)
           (declare (type cma:double-matrix map-arr))
           (let ((max-value 0.0d0))
             (dotimes (row (array-dimension map-arr 0))
               (dotimes (col (array-dimension map-arr 1))
                 (when (> (aref map-arr row col) max-value)
                   (setq max-value (aref map-arr row col)))))
             max-value)))
    (let ((solution-cnt 0))
      (lambda (map)
        (unless (and (>= solution-cnt n-solutions) kipla:*tf*)
          (incf solution-cnt)
          (when (cl-tf:can-transform kipla:*tf* :target-frame "/map" :source-frame "/base_link")
            (let* ((robot-pos (cl-transforms:translation
                               (cl-tf:lookup-transform
                                kipla:*tf* :target-frame "/map" :source-frame "/base_link")))
                   (x (cl-transforms:x robot-pos))
                   (y (cl-transforms:y robot-pos))
                   (map-arr (get-cost-map map))
                   (max-value (max-map-value map-arr)))
              (declare (type cma:double-matrix map-arr))
              (when (> (get-map-value map x y) (* threshold max-value))
                robot-pos))))))))

(defun nav-angle-to-point (p p-ref)
  "Calculates the angle from `p-ref' to face at `p'"
  (cl-transforms:axis-angle->quaternion
   (cl-transforms:make-3d-vector 0 0 1)
   (let ((p-rel (cl-transforms:v- p p-ref)))
     (atan (cl-transforms:y p-rel) (cl-transforms:x p-rel)))))

(def-fact-group location-costmap-utils ()
  (<- (global-fluent-value ?name ?value)
    (symbol-value ?name ?fl)
    (fluent-value ?fl ?value)
    (lisp-pred identity ?value)))

(def-fact-group location-costmap ()
  
  (<- (costmap-size ?width ?height)
    (symbol-value kipla:*map-fl* ?map-fl)
    (fluent-value ?map-fl ?map)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :width ?width)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :height ?height))

  (<- (costmap-resolution 0.05))
  (<- (costmap-padding 0.70))

  (<- (costmap-origin ?x ?y)
    (symbol-value kipla:*map-fl* ?map-fl)
    (fluent-value ?map-fl ?map)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :origin (?x ?y)))

  (<- (occupancy-grid ?msg ?grid)
    (occupancy-grid ?msg ?grid (padding nil)))

  (<- (inverted-occupancy-grid ?msg ?grid)
    (inverted-occupancy-grid ?msg ?grid (padding nil)))
  
  (<- (occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)    
    (lisp-type ?msg nav_msgs-msg:<occupancygrid>)
    (lisp-fun occupancy-grid-msg->occupancy-grid ?msg :padding ?p ?grid))

  (<- (inverted-occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)    
    (lisp-type ?msg nav_msgs-msg:<occupancygrid>)
    (lisp-fun occupancy-grid-msg->occupancy-grid ?msg :padding ?p :invert t ?grid))

  (<- (occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)
    (lisp-type ?msg nav_msgs-msg:<gridcells>)
    (lisp-fun grid-cells-msg->occupancy-grid ?msg ?p ?grid))

  (<- (inverted-occupancy-grid ?msg ?grid (padding ?p))
    (not (bound ?grid))
    (bound ?msg)
    (lisp-type ?msg nav_msgs-msg:<gridcells>)
    (lisp-fun grid-cells-msg->occupancy-grid ?msg ?p ?tmp-grid)
    (lisp-fun invert-occupancy-grid ?tmp-grid ?grid))

  (<- (drivable-location-costmap ?cm)
    (costmap-padding ?padding)
    (global-fluent-value kipla:*map-fl* ?map)
    (global-fluent-value kipla:*table-grid-cells-fl* ?tables)
    (inverted-occupancy-grid ?map ?free-space)
    (occupancy-grid ?map ?static-occupied (padding ?padding))
    (occupancy-grid ?tables ?tables-occupied (padding ?padding))
    (costmap ?cm)
    (lisp-fun cl-transforms:make-3d-vector 0 0 0 ?map-origin)
    (costmap-add-function 10 (make-occupancy-grid-cost-function ?free-space) ?cm)
    (costmap-add-function 9 (make-occupancy-grid-cost-function ?static-occupied :invert t)
                          ?cm)
    (costmap-add-function 8 (make-occupancy-grid-cost-function ?tables-occupied :invert t)
                          ?cm)
    (costmap-add-generator (make-robot-pos-generator 0.1) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (on table))
    (desig-prop ?desig (name ?table-name))
    (annotated-point ?table-name ?_)
    (global-fluent-value kipla:*table-grid-cells-fl* ?table-msg)
    (occupancy-grid ?table-msg ?table-costmap)
    (costmap ?cm)
    (costmap-add-function 5 (make-table-cost-function ?table-name 1.0) ?cm)
    (costmap-add-function 10 (make-occupancy-grid-cost-function ?table-costmap) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (on table))
    (not (desig-prop ?desig (name ?_)))
    (global-fluent-value kipla:*table-grid-cells-fl* ?table-msg)
    (occupancy-grid ?table-msg ?table-costmap)
    (costmap ?cm)
    (costmap-add-function 11 (make-occupancy-grid-cost-function ?table-costmap) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to see))
    (desig-location-prop ?desig ?loc)
    (costmap ?cm)
    (drivable-location-costmap ?cm)
    (costmap-add-function 5 (make-location-cost-function ?loc 0.5) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to reach))
    (desig-location-prop ?desig ?loc)
    (costmap ?cm)
    (drivable-location-costmap ?cm)
    (costmap-add-function 5 (make-location-cost-function ?loc 0.4) ?cm))

  ;; Missing: (for ?obj-type)
  )

(def-fact-group location-orientations ()
  ;; Facts that allow to infer/calculate the correct orientation.
  (<- (desig-orientation ?desig ?point ?orientation)
    (or (desig-prop ?desig (to reach))
        (desig-prop ?desig (to see)))
    (desig-location-prop ?desig ?loc)
    (lisp-fun cl-transforms:origin ?loc ?loc-p)
    (lisp-fun nav-angle-to-point ?loc-p ?point ?orientation)))

(def-fact-group location-z-values ()
  (<- (desig-z-value ?desig ?point ?z)
    (global-fluent-value kipla:*table-height-map-fl* ?table-heightmap)
    (lisp-fun cl-transforms:x ?point ?x)
    (lisp-fun cl-transforms:y ?point ?y)    
    (lisp-fun height-map-lookup ?table-heightmap ?x ?y ?z)))

(def-fact-group location-designators ()
  (<- (loc-desig? ?desig)
    (lisp-pred typep ?desig location-designator))

  (<- (desig-location-prop ?desig ?loc)
    (desig-prop ?desig (obj ?obj))
    (lisp-fun obj-desig-location ?obj ?loc))

  (<- (desig-location-prop ?desig ?loc)
    (desig-prop ?desig (location ?loc-desig))
    (lisp-fun loc-desig-location ?loc-desig ?loc))
  
  (<- (desig-loc ?desig (costmap ?cm))
    (loc-desig? ?desig)
    (bagof ?c (desig-costmap ?desig ?c) ?costmaps)
    (lisp-fun merge-costmaps ?costmaps ?cm))

  (<- (desig-loc ?desig (pose ?p))
    (loc-desig? ?desig)
    (desig-prop ?desig (pose ?p)))

  (<- (desig-loc ?desig (pose ?p))
    (loc-desig? ?desig)
    (desig-prop ?desig (of ?obj))
    (lisp-fun obj-desig-location ?obj ?p)))
