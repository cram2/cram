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
;;; (on table) (on counter)
;;; (to reach) (to see)
;;; (location ?loc)
;;; (pose ?p)
;;; (of ?obj)
;;; (for ?obj-type)

(defun obj-desig-location (obj-desig)
  (when (typep obj-desig 'object-designator)
    (cond ((and (valid obj-desig) (reference obj-desig))
           (object-pose (reference obj-desig)))
          ((desig-prop-value obj-desig 'at)
           (reference (desig-prop-value obj-desig 'at))))))

(defun loc-desig-location (loc-desig)
  (when (and (typep loc-desig 'location-designator)
             loc-desig)
    (reference loc-desig)))

(defun nav-angle-to-point (p p-ref)
  "Calculates the angle from `p-ref' to face at `p'"
  (cl-transforms:axis-angle->quaternion
   (cl-transforms:make-3d-vector 0 0 1)
   (let ((p-rel (cl-transforms:v- p p-ref)))
     (atan (cl-transforms:y p-rel) (cl-transforms:x p-rel)))))

(def-fact-group location-costmap ()
  
  (<- (costmap-size ?width ?height)
    (symbol-value kipla:*map-fl* ?map-fl)
    (fluent-value ?map-fl ?map)
    (lisp-fun occupancy-grid-metadata ?map :key :width ?width)
    (lisp-fun occupancy-grid-metadata ?map :key :height ?height))

  (<- (costmap-resolution 0.05))
  (<- (costmap-padding 0.45))

  (<- (costmap-origin ?x ?y)
    (symbol-value kipla:*map-fl* ?map-fl)
    (fluent-value ?map-fl ?map)
    (lisp-fun occupancy-grid-metadata ?map :key :origin (?x ?y)))

  (<- (drivable-location-costmap ?cm)
    (symbol-value kipla:*map-fl* ?map-fl)
    (symbol-value kipla:*table-costmap-fl* ?table-costmap-fl)
    (fluent-value ?map-fl ?map)
    (fluent-value ?table-costmap-fl ?table-costmap)
    (lisp-pred identity ?map)    
    (lisp-pred identity ?table-costmap)    
    (costmap-padding ?padding)
    (costmap ?cm)
    (costmap-with-function 9 (make-occupancy-grid-cost-function ?table-costmap :invert t :padding ?padding)
                           ?cm)
    (costmap-with-function 10 (make-occupancy-grid-cost-function ?map :invert t :padding ?padding) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (on table))
    (desig-prop ?desig (name ?table-name))
    (symbol-value kipla:*table-costmap-fl* ?table-costmap-fl)
    (fluent-value ?table-costmap-fl ?table-costmap)
    (lisp-pred identity ?table-costmap)    
    (costmap ?cm)
    (costmap-with-function 5 (make-table-cost-function ?table-name 1.0) ?cm)
    (costmap-with-function 10 (make-occupancy-grid-cost-function ?table-costmap) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (on table))
    (not (desig-prop ?desig (name ?_)))
    (symbol-value kipla:*table-costmap-fl* ?table-costmap-fl)
    (fluent-value ?table-costmap-fl ?table-costmap)
    (lisp-pred identity ?table-costmap)
    (costmap ?cm)
    (costmap-with-function 10 (make-occupancy-grid-cost-function ?table-costmap) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to see))
    (desig-location-prop ?desig ?loc)
    (costmap ?cm)
    (drivable-location-costmap ?cm)
    (costmap-with-function 5 (make-location-cost-function ?loc 0.4) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to reach))
    (desig-location-prop ?desig ?loc)
    (costmap ?cm)
    (drivable-location-costmap ?cm)
    (costmap-with-function 5 (make-location-cost-function ?loc 0.2) ?cm))

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

  (<- (loc-desig ?desig (pose ?p))
    (loc-desig? ?desig)
    (desig-prop ?desig (pose ?p)))

  (<- (loc-desig ?desig (pose ?p))
    (loc-desig? ?desig)
    (desig-prop ?desig (of ?obj))
    (lisp-fun object-desig-location ?obj ?p)))
