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
;;; By itself, the file will do nothing, as it does not define any costmaps,
;;; so to use it you have to
;;; also load a file describing costmaps and providing them with prolog rules such as
;;;   (<- (desig-costmap ?desig ?cm)
;;;     (costmap ?cm)
;;;     (desig-prop ?desig (:type :visible))
;;;     ...)
;;; examples are in table_costmap and semantic_map_costmap

(defun make-angle-to-point-generator (position
                                      &key (samples 1) sample-step sample-offset)
  "Returns a function that takes an X and Y coordinate and returns a lazy-list of
quaternions to face towards `position'"
  (make-orientation-generator (alexandria:rcurry #'angle-to-point-direction position)
                              :samples samples :sample-step sample-step
                              :sample-offset sample-offset))

(defun angle-to-point-direction (x y position)
  "Returns a function that takes an X and Y coordinate and returns a
quaternion to face towards `position'"
  (let* ((point (etypecase position
                  (cl-transforms:pose (cl-transforms:origin position))
                  (cl-transforms:3d-vector position)))
         (p-rel (cl-transforms:v- point (cl-transforms:make-3d-vector x y 0))))
    (cl-transforms:normalize-angle
     (atan (cl-transforms:y p-rel) (cl-transforms:x p-rel)))))

(defun make-orientation-generator (main-direction-function
                                   &key (samples 1) sample-step sample-offset)
  "Returns a lazy-list of quaternions around the main direction generated by
`main-direction-function' if PREVIOUS-ORIENTATION is NIL, otherwise PREVIOUS-ORIENTATION.
`samples' indicates how many rotations should be returned.
If the value is greater than 1, the samples' orientations differ by `sample-step'.
The order is random, so the main direction is NOT prioritized."
  (flet ((make-angles (samples sample-step)
           (alexandria:shuffle
            (cons
             (cl-transforms:euler->quaternion :az (or sample-offset 0.0d0))
             (loop
               for angle from sample-step
                 below (* sample-step (ceiling (/ samples 2)))
               by sample-step
               appending (list (cl-transforms:euler->quaternion :az angle)
                               (cl-transforms:euler->quaternion :az (- angle))))))))
    (let ((angle-differences (if (and sample-step (> samples 1))
                                 (make-angles samples sample-step)
                                 (list (cl-transforms:euler->quaternion
                                        :az (or sample-offset 0.0d0))))))
      (lambda (x y previous-orientations)
        (or previous-orientations
            (let ((angle (funcall main-direction-function x y)))
              (lazy-mapcar (lambda (angle-difference)
                             (cl-transforms:q*
                              angle-difference
                              (cl-transforms:euler->quaternion :az angle)))
                           angle-differences)))))))

(defun 2d-pose-covariance (positions &optional (minimal-variance 0.1))
  (let* ((points (force-ll (lazy-mapcar #'get-point positions)))
         (points-length (length points))
         (mean-x (/ (reduce (lambda (previous point)
                              (+ previous (cl-transforms:x point)))
                            points :initial-value 0.0d0)
                    points-length))
         (mean-y (/ (reduce (lambda (previous point)
                              (+ previous (cl-transforms:y point)))
                            points :initial-value 0.0d0)
                    points-length))
         (result (make-array '(2 2) :element-type 'double-float :initial-element 0.0d0)))
    (dolist (point points)
      (incf (aref result 0 0) (max
                               (* (- (cl-transforms:x point) mean-x)
                                  (- (cl-transforms:x point) mean-x))
                               minimal-variance))
      (incf (aref result 0 1) (* (- (cl-transforms:x point) mean-x)
                                 (- (cl-transforms:y point) mean-y)))
      (incf (aref result 1 0) (aref result 0 1))
      (incf (aref result 1 1) (max
                               (* (- (cl-transforms:y point) mean-y)
                                  (- (cl-transforms:y point) mean-y))
                               minimal-variance)))
    (dotimes (y 2)
      (dotimes (x 2)
        (setf (aref result y x) (/ (aref result y x) points-length))))
    (list (cl-transforms:make-3d-vector mean-x mean-y 0.0d0)
          result)))

(def-fact-group location-costmap-utils (desig-costmap)

  ;; default costmap resolver to define the predicate
  (<- (desig-costmap ?designator ?costmap)
    (fail))

  (<- (merged-desig-costmap ?desig ?cm)
    ;; bagof collects all true solutions for c into costmaps
    (bagof ?c (desig-costmap ?desig ?c) ?costmaps)
    (lisp-fun merge-costmaps ?costmaps ?cm))

  (<- (costmap-samples ?cm ?solutions)
    (lisp-fun costmap-samples ?cm ?solutions))

  ;; TODO: This fact belongs into a package for CRAM_PL based desig utils
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

;; costmap metadata default predicate definition
(def-fact-group location-costmap-metadata (costmap-size
                                           costmap-origin
                                           costmap-resolution
                                           orientation-samples
                                           orientation-sample-step
                                           reachability-orientation-offset
                                           costmap-padding
                                           costmap-manipulation-padding
                                           costmap-in-reach-distance
                                           costmap-reach-minimal-distance
                                           visibility-costmap-size)

  ;; costmap dimensions in meters, diameter not radius
  ;; depends on the size of the environment the robot operates in
  (<- (costmap-size ?environment-name ?width ?height)
    (fail))

  ;; the coordinate of the left bottom corner of the costmap in meters in map frame
  ;; depends on the environment
  (<- (costmap-origin ?environment-name ?x ?y)
    (fail))

  ;; size of one costmap cell in meters
  ;; the smaller the number the higher the accuracy and the slower the algorithms
  (<- (costmap-resolution ?environment-name ?resolution)
    (fail))

  ;; number of orientations to generate for orientation generators
  ;; mostly used in gaussian-cm
  (<- (orientation-samples ?robot-name ?samples)
    (fail))

  ;; when generating ORIENTATION-SAMPLES number of samples,
  ;; what should be the distance between different samples in radians
  (<- (orientation-sample-step ?robot-name ?step-in-radian)
    (fail))

  ;; when generating ORIENTATION-SAMPLES number of samples,
  ;; what should be the angle offset from looking directly at target
  (<- (reachability-orientation-offset ?robot-name ?offset-in-radian)
    (fail))

  ;; padding offset from obstacles, depends on the robot base dimension
  ;; mostly used for visibility
  (<- (costmap-padding ?robot-name ?padding-in-meters)
    (fail))

  ;; padding offset from obstacles when manipulating objects
  ;; can be different from COSTMAP-PADDING if the arm needs extra padding when grasping
  (<- (costmap-manipulation-padding ?robot-name ?padding-in-meters)
    (fail))

  ;; maximum length at which a robot can reach an object from its base location
  (<- (costmap-in-reach-distance ?robot-name ?distance-in-meters)
    (fail))

  ;; minimum distance from which the robot can reach an object from its base location
  ;; actually should probably be merged with COSTMAP-MANIPULATION-PADDING
  ;; at least the numbers should be very close to each other
  (<- (costmap-reach-minimal-distance ?robot-name ?distance-in-meters)
    (fail))

  ;; maximum distance from which the robot can perceive an object
  ;; i.e. the radius of the visibility gaussian costmap
  (<- (visibility-costmap-size ?robot-name ?radius-in-meters)
    (fail)))
