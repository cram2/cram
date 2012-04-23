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

(in-package :bullet-reasoning)

(defparameter *grasps* `((:top . ,(cl-transforms:euler->quaternion :ay (/ pi -2)))
                         (:left . ,(cl-transforms:euler->quaternion :az (/ pi 2)))
                         (:right . ,(cl-transforms:euler->quaternion :az (/ pi -2)))
                         (:front . ,(cl-transforms:make-identity-rotation))))

(defun get-grasp (grasp side)
  (ecase grasp
    (:top (cdr (assoc :top *grasps*)))
    (:front (cdr (assoc :front *grasps*)))
    (:side (cdr (assoc side *grasps*)))))

(defun calculate-orientation-in-robot (robot orientation-in-robot)
  "Calculates the orientation of `orientation-in-robot' which is
relative to the robot in world coordinates."
  (cl-transforms:q*
   (cl-transforms:orientation (pose robot))
   orientation-in-robot))

(defun calculate-object-tool-length (object &key (minimal-tool-length 0.20))
  "Calculates the tool length for the object `object' by taking the
  maximum of `minimal-tool-length', the bounding box width minus the
  minimal tool length and the bounding box height minus the minimal
  tool length."
  (declare (type object object)
           (type number minimal-tool-length))
  (let ((bounding-box-dimensions (bounding-box-dimensions (aabb object))))
    (max minimal-tool-length
         (- (cl-transforms:x bounding-box-dimensions) minimal-tool-length)
         (- (cl-transforms:y bounding-box-dimensions) minimal-tool-length))))

(defun object-reachable-p (robot obj
                           &key side (grasp :top)
                             (tool-length (calculate-object-tool-length obj)))
  (declare (type robot-object robot)
           (type object obj))
  (point-reachable-p
   robot (pose obj) :grasp grasp :side side :tool-length tool-length))

(defun point-reachable-p (robot point
                          &key side (grasp :top) (tool-length 0.20))
  (declare (type robot-object robot)
           (type (or cl-transforms:3d-vector
                     cl-transforms:pose)
                 point))
  (lazy-car
   (reach-pose-ik
    robot (cl-transforms:make-pose
           (etypecase point
             (cl-transforms:3d-vector point)
             (cl-transforms:pose (cl-transforms:origin point)))
           (calculate-orientation-in-robot
            robot (cl-transforms:make-identity-rotation)))
   :tool-frame (cl-transforms:make-pose
                (cl-transforms:make-3d-vector
                 tool-length 0.0 0.0)
                (get-grasp grasp side))
    :side side)))

(defun pose-reachable-p (robot pose
                           &key
                             side (tool-frame
                                   (cl-transforms:make-pose
                                    (cl-transforms:make-3d-vector 0.20 0.0 0.0)
                                    (cl-transforms:make-identity-rotation))))
  (declare (type robot-object robot)
           (type cl-transforms:pose pose)
           (type cl-transforms:pose tool-frame))
  (lazy-car
   (reach-pose-ik
    robot pose :tool-frame tool-frame :side side)))

(defun reach-object-ik (robot obj
                        &key side (grasp :top)
                          (tool-length (calculate-object-tool-length obj)))
  (declare (type robot-object robot)
           (type object obj))
  (reach-pose-ik
   robot (cl-transforms:copy-pose
          (pose obj) :new-orientation (calculate-orientation-in-robot
                                       robot (cl-transforms:make-identity-rotation)))
   :tool-frame (cl-transforms:make-pose
                (cl-transforms:make-3d-vector
                 tool-length 0.0 0.0)
                (get-grasp grasp side))
   :side side))

(defun reach-pose-ik (robot pose
                      &key
                        side (tool-frame (cl-transforms:make-pose
                                          (cl-transforms:make-3d-vector 0.20 0.0 0.0)
                                          (cl-transforms:make-identity-rotation))))
  (declare (type robot-object robot)
           (type cl-transforms:pose pose)
           (type cl-transforms:pose tool-frame))
  (let ((pose-transform-in-robot (cl-transforms:transform*
                                  (cl-transforms:transform-inv
                                   (cl-transforms:reference-transform
                                    (pose robot)))
                                  (cl-transforms:reference-transform pose)))
        (reference-frame (cl-urdf:name (cl-urdf:root-link (slot-value robot 'urdf)))))
    (get-ik
     robot (tf:make-pose-stamped
            reference-frame 0.0
            (cl-transforms:translation pose-transform-in-robot)
            (cl-transforms:rotation pose-transform-in-robot))
     :ik-namespace (ecase side
                     (:right "reasoning/pr2_right_arm_kinematics")
                     (:left "reasoning/pr2_left_arm_kinematics"))
     :tool-frame tool-frame
     :fixed-frame reference-frame)))
