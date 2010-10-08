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

(in-package :table-costmap)

(defmethod costmap-generator-name->score ((name (eql 'reachable-space)))
  4)

(defmethod costmap-generator-name->score ((name (eql 'location-neighborhood)))
  5)

(defmethod costmap-generator-name->score ((name (eql 'named-table)))
  5)

(defmethod costmap-generator-name->score ((name (eql 'all-tables)))
  10)

(defmethod costmap-generator-name->score ((name (eql 'tables-occupied)))
  8)

(defmethod costmap-generator-name->score ((name (eql 'static-occupied)))
  9)

(defmethod costmap-generator-name->score ((name (eql 'free-space)))
  10)

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
        (unless (and (>= solution-cnt n-solutions) *tf*)
          (incf solution-cnt)
          (when (cl-tf:can-transform *tf* :target-frame "/map" :source-frame "/base_link")
            (let* ((robot-pos (cl-transforms:translation
                               (cl-tf:lookup-transform
                                *tf* :target-frame "/map" :source-frame "/base_link")))
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

;;; Requires the following predicates to be set:
;;;
;;; (costmap-size ?w ?h)
;;; (costmap-manipulation-padding ?p)
;;; (costmap-padding ?p)
;;; (costmap-in-reach-padding ?p)
(def-fact-group table-costmap (desig-costmap
                               desig-orientation
                               desig-z-value
                               drivable-location-costmap)

  (<- (drivable-location-costmap ?cm ?padding)
    (global-fluent-value *map-fl* ?map)
    (global-fluent-value *table-grid-cells-fl* ?tables)
    (inverted-occupancy-grid ?map ?free-space)
    (occupancy-grid ?map ?static-occupied (padding ?padding))
    (occupancy-grid ?tables ?tables-occupied (padding ?padding))
    (costmap ?cm)
    (costmap-add-function free-space (make-occupancy-grid-cost-function ?free-space) ?cm)
    (costmap-add-function static-occupied (make-occupancy-grid-cost-function ?static-occupied :invert t)
                          ?cm)
    (costmap-add-function tables-occupied (make-occupancy-grid-cost-function ?tables-occupied :invert t)
                          ?cm)
    (costmap-add-generator (make-robot-pos-generator 0.1) ?cm))

  (<- (in-reach-costmap ?cm ?padding ?reaching-distance)
    (bagof ?one-drivable-cm (drivable-location-costmap ?one-drivable-cm ?padding)
           ?drivable-costmaps)
    (lisp-fun merge-costmaps ?drivable-costmaps ?drivable-costmap)
    (costmap ?cm)
    (costmap-add-function reachable-space (make-padded-costmap-cost-function
                                           ?drivable-costmap ?reaching-distance)
                          ?cm))
  
  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (on table))
    (desig-prop ?desig (name ?table-name))
    (annotated-point ?table-name ?_)
    (global-fluent-value *table-grid-cells-fl* ?table-msg)
    (occupancy-grid ?table-msg ?table-costmap)
    (costmap ?cm)
    (costmap-add-function named-table (make-table-cost-function ?table-name) ?cm)
    (costmap-add-function all-tables (make-occupancy-grid-cost-function ?table-costmap) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (on table))
    (not (desig-prop ?desig (name ?_)))
    (global-fluent-value *table-grid-cells-fl* ?table-msg)
    (occupancy-grid ?table-msg ?table-costmap)
    (costmap ?cm)
    (costmap-add-function all-tables (make-occupancy-grid-cost-function ?table-costmap) ?cm))
  
  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to see))
    (costmap ?cm)
    (costmap-padding ?padding)
    (drivable-location-costmap ?cm ?padding))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to see))
    (costmap ?cm)
    (desig-location-prop ?desig ?loc)
    (costmap-add-function location-neighborhood (make-location-cost-function ?loc 0.5) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (in reach))
    (costmap ?cm)
    (costmap-manipulation-padding ?padding)
    (costmap-in-reach-padding ?in-reach-padding)
    (in-reach-costmap ?cm ?padding ?in-reach-padding))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to reach))
    (desig-location-prop ?desig ?loc)
    (costmap ?cm)
    (costmap-manipulation-padding ?padding)
    (drivable-location-costmap ?cm ?padding)
    (costmap-add-function location-neighborhood (make-location-cost-function ?loc 0.4) ?cm))

    ;; Missing: (for ?obj-type)

  (<- (desig-z-value ?desig ?point ?z)
    (global-fluent-value *table-height-map-fl* ?table-heightmap)
    (lisp-fun cl-transforms:x ?point ?x)
    (lisp-fun cl-transforms:y ?point ?y)    
    (lisp-fun height-map-lookup ?table-heightmap ?x ?y ?z))

  (<- (desig-orientation ?desig ?point ?orientation)
    (or (desig-prop ?desig (to reach))
        (desig-prop ?desig (to see)))
    (desig-location-prop ?desig ?loc)
    (lisp-fun cl-transforms:origin ?loc ?loc-p)
    (lisp-fun nav-angle-to-point ?loc-p ?point ?orientation)))
