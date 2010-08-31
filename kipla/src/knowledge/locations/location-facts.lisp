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
  (when (typep obj-desig 'object-designator)
    (cond ((and (valid obj-desig) (reference obj-desig))
           (object-pose (reference obj-desig)))
          ((desig-prop-value obj-desig 'at)
           (reference (desig-prop-value obj-desig 'at))))))

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
          (when (cl-tf:can-transform kipla:*tf* :source-frame "/map" :target-frame "/base_link")
            (let* ((robot-pos (cl-transforms:translation
                               (cl-tf:lookup-transform
                                kipla:*tf* :source-frame "/map" :target-frame "/base_link")))
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

(def-fact-group location-orientations (desig-orientation)
  ;; Facts that allow to infer/calculate the correct orientation.
  (<- (desig-orientation ?desig ?point ?orientation)
    (or (desig-prop ?desig (to reach))
        (desig-prop ?desig (to see)))
    (desig-location-prop ?desig ?loc)
    (lisp-fun cl-transforms:origin ?loc ?loc-p)
    (lisp-fun nav-angle-to-point ?loc-p ?point ?orientation)))

(def-fact-group costmap-metadata ()
    (<- (costmap-size ?width ?height)
    (symbol-value table-costmap:*map-fl* ?map-fl)
    (fluent-value ?map-fl ?map)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :width ?width)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :height ?height))

  (<- (costmap-resolution 0.05))
  (<- (costmap-padding 0.85))

  (<- (costmap-origin ?x ?y)
    (symbol-value table-costmap:*map-fl* ?map-fl)
    (fluent-value ?map-fl ?map)
    (lisp-fun occupancy-grid-msg-metadata ?map :key :origin (?x ?y))))

(def-fact-group location-z-values (desig-z-value))

(def-fact-group location-designators (desig-loc)
  (<- (loc-desig? ?desig)
    (lisp-pred typep ?desig location-designator))

  (<- (desig-location-prop ?desig ?loc)
    (desig-prop ?desig (obj ?obj))
    (lisp-fun obj-desig-location ?obj ?loc))

  (<- (desig-location-prop ?desig ?loc)
    (desig-prop ?desig (location ?loc-desig))
    (lisp-fun loc-desig-location ?loc-desig ?loc))
  
  (<- (desig-loc ?desig (pose ?p))
    (loc-desig? ?desig)
    (desig-prop ?desig (pose ?p)))

  (<- (desig-loc ?desig (pose ?p))
    (loc-desig? ?desig)
    (desig-prop ?desig (of ?obj))
    (lisp-fun object-desig-location ?obj ?p)))
