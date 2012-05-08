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

(defparameter *orientation-samples* 5)
(defparameter *orientation-sample-step* (/ pi 18))

(defmethod costmap-generator-name->score ((name (eql 'pose-distribution)))
  5)

(defclass pose-distribution-range-include-generator () ())

(defclass pose-distribution-range-exclude-generator () ())

(defmethod costmap-generator-name->score
    ((name pose-distribution-range-include-generator))
  7)

(defmethod costmap-generator-name->score
    ((name pose-distribution-range-exclude-generator))
  6)

(defun make-angle-to-point-generator (position &key (samples 1) sample-step)
  "Returns a function that takes an X and Y coordinate and returns a
quaternion to face towards `pose' if PREVIOUS-ORIENTATION is not NIL,
otherwise PREVIOUS-ORIENTATION. `samples' indicates how many rotations
should be returned. If the value is greater than 1, the samples's
orientations differ by `sample-step'."
  (flet ((make-angles (samples sample-step)
           (cons (cl-transforms:euler->quaternion :az 0.0d0)
                 (loop for angle from sample-step
                         below (* sample-step (ceiling (/ samples 2)))
                           by sample-step
                       appending (list (cl-transforms:euler->quaternion :az angle)
                                       (cl-transforms:euler->quaternion :az (- angle)))))))
    (let ((point (etypecase position
                   (cl-transforms:pose (cl-transforms:origin position))
                   (cl-transforms:3d-vector position)))
          (angle-differences (if (and sample-step (> samples 1))
                                 (make-angles samples sample-step)
                                 (list (cl-transforms:make-identity-rotation)))))
      (lambda (x y previous-orientations)
        (or previous-orientations
            (let* ((p-rel (cl-transforms:v-
                           point (cl-transforms:make-3d-vector x y 0)))
                   (angle (atan (cl-transforms:y p-rel) (cl-transforms:x p-rel))))
              (lazy-mapcar (lambda (angle-difference)
                             (cl-transforms:q*
                              angle-difference
                              (cl-transforms:euler->quaternion :az angle)))
                           angle-differences)))))))

(defun 2d-pose-covariance (poses &optional (minimal-variance 0.1))
  (let* ((poses (force-ll poses))
         (poses-length (length poses))
         (mean-x (/ (reduce (lambda (previous pose)
                              (+ previous (cl-transforms:x
                                           (cl-transforms:origin pose))))
                            poses :initial-value 0.0d0)
                    poses-length))
         (mean-y (/ (reduce (lambda (previous pose)
                              (+ previous (cl-transforms:y
                                           (cl-transforms:origin pose))))
                            poses :initial-value 0.0d0)
                    poses-length))
         (result (make-array '(2 2) :element-type 'double-float :initial-element 0.0d0)))
    (dolist (pose poses)
      (incf (aref result 0 0) (max
                               (* (- (cl-transforms:x
                                      (cl-transforms:origin pose)) mean-x)
                                  (- (cl-transforms:x
                                      (cl-transforms:origin pose)) mean-x))
                               minimal-variance))
      (incf (aref result 0 1) (* (- (cl-transforms:x
                                     (cl-transforms:origin pose)) mean-x)
                                 (- (cl-transforms:y
                                     (cl-transforms:origin pose)) mean-y)))
      (incf (aref result 1 0) (aref result 0 1))
      (incf (aref result 1 1) (max
                               (* (- (cl-transforms:y
                                      (cl-transforms:origin pose)) mean-y)
                                  (- (cl-transforms:y
                                      (cl-transforms:origin pose)) mean-y))
                               minimal-variance)))
    (dotimes (y 2)
      (dotimes (x 2)
        (setf (aref result y x) (/ (aref result y x) poses-length))))
    (list (cl-transforms:make-3d-vector mean-x mean-y 0.0d0)
          result)))

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
    (bagof ?pose (desig-location-prop ?desig ?pose) ?poses)
    (costmap ?cm)
    (lisp-fun 2d-pose-covariance ?poses 0.5 (?mean ?covariance))
    (costmap-add-function pose-distribution (make-gauss-cost-function ?mean ?covariance) ?cm)
    (symbol-value *orientation-samples* ?samples)
    (symbol-value *orientation-sample-step* ?sample-step)
    (costmap-add-orientation-generator
     (make-angle-to-point-generator ?mean :samples ?samples :sample-step ?sample-step)
     ?cm))
  
  (<- (desig-costmap ?desig ?cm)
    (reachability-designator ?desig)
    (bagof ?pose (designator-reach-pose ?desig ?pose) ?poses)
    (costmap ?cm)
    (lisp-fun 2d-pose-covariance ?poses 0.5 (?mean ?covariance))
    (costmap-in-reach-distance ?distance)
    (costmap-reach-minimal-distance ?minimal-distance)
    (forall
     (member ?pose ?poses)
     (and
      (lisp-fun cl-transforms:origin ?pose ?point)
      (instance-of pose-distribution-range-include-generator
                   ?include-generator-id)
      (costmap-add-function
       ?include-generator-id
       (make-range-cost-function ?point ?distance) ?cm)
      (instance-of pose-distribution-range-exclude-generator
                   ?exclude-generator-id)
      (costmap-add-function
       ?exclude-generator-id
       (make-range-cost-function ?point ?minimal-distance :invert t)
       ?cm)))
    (costmap-add-function pose-distribution (make-gauss-cost-function ?mean ?covariance) ?cm)
    (symbol-value *orientation-samples* ?samples)
    (symbol-value *orientation-sample-step* ?sample-step)
    (costmap-add-orientation-generator
     (make-angle-to-point-generator ?mean :samples ?samples :sample-step ?sample-step)
     ?cm))

  (<- (merged-desig-costmap ?desig ?cm)
    ;; bagof collects all true solutions for c into costmaps
    (bagof ?c (desig-costmap ?desig ?c) ?costmaps)
    (lisp-fun merge-costmaps ?costmaps ?cm))

  (<- (costmap-samples ?cm ?solutions)
    (lisp-fun costmap-samples ?cm ?solutions)))

(def-fact-group reachability-designators ()

  (<- (reachability-designator ?designator)
    (desig-prop ?designator (to reach)))

  (<- (reachability-designator ?designator)
    (desig-prop ?designator (to execute))
    (desig-prop ?designator (action ?_)))

  (<- (designator-reach-pose ?designator ?pose)
    (reachability-designator ?designator)
    (desig-prop ?designator (pose ?pose)))

  (<- (designator-reach-pose ?designator ?point)
    (reachability-designator ?designator)
    (or (desig-prop ?designator (object ?object))
        (desig-prop ?designator (obj ?object)))
    (desig-location-prop ?object ?pose)
    (lisp-fun cl-transforms:origin ?pose ?point))

  (<- (designator-reach-pose ?designator ?pose)
    (reachability-designator ?designator)
    (desig-prop ?designator (location ?location))
    (desig-location-prop ?location ?pose))

  (<- (designator-reach-pose ?designator ?pose)
    (reachability-designator ?designator)
    (desig-prop ?designator (to execute))
    (desig-prop ?designator (action ?action))
    (plan-knowledge:trajectory-point ?action ?pose)))
