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

(defun make-aligned-orientation-generator (reference-pose pose)
  (lambda (x y orientation)
    (declare (ignore orientation))
    (flet ((yaw-between-points (point-1 point-2)
             (let ((offset (cl-transforms:v- point-2 point-1)))
               (atan (cl-transforms:y offset)
                     (cl-transforms:x offset)))))
      (let* ((direct-angle-to-pose (abs (yaw-between-points
                                         (cl-transforms:make-3d-vector x y 0)
                                         (cl-transforms:origin pose))))
             (current-orientation (cl-transforms:orientation reference-pose))
             (best-aligned-orientation current-orientation)
             (best-aligned-angle (abs (- (nth-value
                                          1 (cl-transforms:quaternion->axis-angle
                                             best-aligned-orientation))
                                         direct-angle-to-pose)))
             (90-degree (cl-transforms:euler->quaternion :az (/ pi 2))))
        (dotimes (i 3 best-aligned-orientation)
          (setf current-orientation (cl-transforms:q* current-orientation 90-degree))
          (let ((current-angle-to-point
                  (abs (- (nth-value 1 (cl-transforms:quaternion->axis-angle current-orientation))
                          direct-angle-to-pose))))
            (when (< current-angle-to-point best-aligned-angle)
              (setf best-aligned-orientation current-orientation)
              (setf best-aligned-angle current-angle-to-point))))))))

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
    (desig-prop ?desig (obj ?obj))
    (desig-location-prop ?desig ?pose)
    (costmap ?cm)
    (bullet-world ?world)
    (object ?world ?obj)
    (contact ?world ?obj ?sem-map ?contacting-link)
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
    (assert-object-pose ?robot ?robot-pose)
    (not (contact ?robot ?_))
    (-> (desig-prop ?desig (side ?side)) (true) (true))
    (desig-prop ?desig (obj ?obj))
    (reachable ?robot ?obj ?side)
    (blocking ?robot ?obj ?side ()))
    
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
    (assert-object-pose ?robot ?robot-pose)
    (not (contact ?robot ?_))
    (head-pointing-at ?robot ?obj-pose)
    (link-pose ?robot ?cam-frame ?cam-pose)
    (desig-prop ?desig (obj ?obj))
    (visible ?cam-pose ?obj))

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
