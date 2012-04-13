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
;;;

(in-package :btr-desig)

(defvar *max-location-samples* 50)

(defmethod costmap-generator-name->score ((name (eql 'reachable-from-space)))
  5)

(defmethod costmap-generator-name->score ((name (eql 'reachable-from-weighted)))
  4)

(defmethod costmap-generator-name->score ((name (eql 'pose-distribution)))
  5)

(defun make-aligned-orientation-generator (reference-pose pose)
  (flet ((yaw-between-points (point-1 point-2)
           (let ((offset (cl-transforms:v- point-2 point-1)))
             (atan (cl-transforms:y offset) (cl-transforms:x offset)))))
    (labels ((find-closest-angle (reference-angle angles &optional current-best-angle)
               (cond ((and angles (not current-best-angle))
                      (find-closest-angle
                       reference-angle (cdr angles) (car angles)))
                     (angles
                      (find-closest-angle
                       reference-angle (cdr angles)
                       (if (< (abs (cl-transforms:normalize-angle
                                    (- reference-angle (car angles))))
                              (abs (cl-transforms:normalize-angle
                                    (- reference-angle current-best-angle))))
                           (car angles)
                           current-best-angle)))
                     (t (cl-transforms:euler->quaternion :az current-best-angle)))))
      (lambda (x y orientation)
        (declare (ignore orientation))
        (find-closest-angle
         (yaw-between-points
          (cl-transforms:make-3d-vector x y 0)
          (cl-transforms:origin pose))
         (loop with pi/2 = (/ pi 2)
               with reference-angle = (cl-transforms:get-yaw (cl-transforms:orientation reference-pose))
               for i from 0 below 4 collecting (cl-transforms:normalize-angle
                                                (+ reference-angle (* i pi/2)))))))))

(defun 2d-pose-covariance (poses)
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
      (incf (aref result 0 0) (* (- (cl-transforms:x
                                     (cl-transforms:origin pose)) mean-x)
                                 (- (cl-transforms:x
                                     (cl-transforms:origin pose)) mean-x)))
      (incf (aref result 0 1) (* (- (cl-transforms:x
                                     (cl-transforms:origin pose)) mean-x)
                                 (- (cl-transforms:y
                                     (cl-transforms:origin pose)) mean-y)))
      (incf (aref result 1 0) (aref result 0 1))
      (incf (aref result 1 1) (* (- (cl-transforms:y
                                     (cl-transforms:origin pose)) mean-y)
                                 (- (cl-transforms:y
                                     (cl-transforms:origin pose)) mean-y))))
    (dotimes (y 2)
      (dotimes (x 2)
        (setf (aref result y x) (/ (aref result y x) poses-length))))
    (list (cl-transforms:make-pose
           (cl-transforms:make-3d-vector mean-x mean-y 0.0d0)
           (cl-transforms:make-identity-rotation))
          result)))

(def-fact-group bullet-reasoning-location-desig (desig-costmap
                                                 desig-loc
                                                 desig-location-prop)
  (<- (desig-costmap ?desig ?cm)
    (costmap ?cm)
    (desig-prop ?desig (reachable-from ?pose))
    (lisp-fun cl-transforms:origin ?pose ?point)
    (costmap-in-reach-padding ?distance)
    (costmap-add-function reachable-from-space
                          (make-range-cost-function ?point ?distance)
                          ?cm)
    (costmap-add-function reachable-from-weighted
                          (make-location-cost-function ?pose ?distance)
                          ?cm))

  (<- (desig-costmap ?desig ?cm)
    (or (desig-prop ?desig (to see))
        (desig-prop ?desig (to reach)))
    (bagof ?pose (or
                  (desig-location-prop ?desig ?pose)
                  (desig-prop ?desig (pose ?pose)))
           ?poses)
    (costmap ?cm)
    (lisp-fun 2d-pose-covariance ?poses (?mean ?covariance))
    (costmap-add-function pose-distribution (make-location-cost-function ?mean ?covariance) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (or (desig-prop ?desig (to see))
        (desig-prop ?desig (to reach)))
    (desig-prop ?desig (obj ?obj))
    (desig-location-prop ?desig ?pose)
    (costmap ?cm)
    (bullet-world ?world)
    (object ?world ?obj)
    (once (contact ?world ?obj ?sem-map ?contacting-link))
    (link-pose ?sem-map ?contacting-link ?reference-pose)
    (costmap-add-orientation-generator
     (make-aligned-orientation-generator ?reference-pose ?pose)
     ?cm))

  (<- (desig-location-prop ?desig ?loc)
    (or (loc-desig? ?desig)
        (obj-desig? ?desig))
    (desig-prop ?desig (obj ?o))
    (object ?o)
    (pose ?o ?loc))

  (<- (desig-location-prop ?o ?loc)
    (object ?o)
    (pose ?o ?loc))  

  (<- (desig-check-to-reach ?desig ?robot-pose)
    (bullet-world ?w)
    (robot ?robot)
    (assert (object-pose ?robot ?robot-pose))
    (not (contact ?robot ?_))
    (-> (desig-prop ?desig (side ?side)) (true) (true))
    (forall (desig-prop ?desig (obj ?obj))
            (reachable ?robot ?obj ?side)))
    
  (<- (location-valid
       ?desig ?pose
       (desig-check-to-see ?desig ?pose))
    (desig-prop ?desig (to see))
    (desig-prop ?desig (obj ?obj)))

  (<- (location-valid
       ?desig ?pose
       (desig-check-to-reach ?desig ?pose))
    (desig-prop ?desig (to reach))
    (desig-prop ?desig (obj ?obj)))

  (<- (desig-check-to-see ?desig ?robot-pose)
    (bullet-world ?w)
    (robot ?robot)
    (camera-frame ?cam-frame)
    (desig-location-prop ?desig ?obj-pose)
    (assert (object-pose ?robot ?robot-pose))
    (not (contact ?robot ?_))
    (head-pointing-at ?robot ?obj-pose)
    (desig-prop ?desig (obj ?obj))
    (visible ?robot ?obj))

  (<- (btr-desig-solution-valid ?desig ?solution)
    (bullet-world ?w)
    (findall ?check (location-valid ?desig ?solution ?check)
             ?checks)
    (with-stored-world ?w
      (forall (member ?check ?checks) (call ?check))))
  
  (<- (btr-desig-solutions ?desig ?points)
    (merged-desig-costmap ?desig ?cm)
    (debug-costmap ?cm 0.0)
    (costmap-samples ?cm ?solutions)
    (symbol-value *max-location-samples* ?max-samples)
    (take ?max-samples ?solutions ?n-solutions)
    (bagof ?point (and
                   (member ?point ?n-solutions)
                   (btr-desig-solution-valid ?desig ?point))
           ?points)))
