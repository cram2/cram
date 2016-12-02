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

(in-package :cram-bullet-reasoning)

(defgeneric pose-reachable-p (robot pose &key side tool-frame)
  (:documentation "Checks if a pose is reachable for the robot
`robot'. `side' is used to get the namespace of the IK service and
`tool-frame' is the tool transformation to use."))

(defgeneric reach-pose-ik (robot pose &key side tool-frame)
  (:documentation "Returns the IK solution for `robot' to reach
`pose'. Returns NUL if the pose is unreachable."))

(defun calculate-orientation-in-robot (robot orientation-in-robot)
  "Calculates the orientation of `orientation-in-robot' which is
relative to the robot in world coordinates."
  (cl-transforms:q*
   (cl-transforms:orientation (pose robot))
   orientation-in-robot))

(defun calculate-object-tool-length
    (object &key (minimal-tool-length (get-tool-length)))
  "Calculates the tool length for the object `object' by taking the
maximum of `minimal-tool-length', the bounding box width minus the
minimal tool length and the bounding box height minus the minimal
tool length."
  (declare (type object object)
           (type number minimal-tool-length))
  (calculate-bounding-box-tool-length
   (bounding-box-dimensions (aabb object))
   :minimal-tool-length minimal-tool-length))

(defun translate-grasp (grasp side)
  "Returns the `grasp' that is either called `grasp' or has an alias
`side'."
  (car (get-grasps grasp (lambda (grasp-name)
                           (or (eq grasp-name grasp)
                               (eq grasp-name side))))))

(defun object-reachable-p (robot obj
                           &key side (grasp :top)
                             (tool-length (calculate-object-tool-length obj)))
  (declare (type robot-object robot)
           (type object obj))
  (point-reachable-p
   robot (pose obj) :grasp grasp :side side :tool-length tool-length))

(defun point-reachable-p (robot point
                          &key side (grasp :top) (tool-length (get-tool-length)))
  (declare (type robot-object robot)
           (type (or cl-transforms:3d-vector
                     cl-transforms:pose)
                 point))
  (pose-reachable-p
   robot
   (cl-transforms:make-pose
    (etypecase point
      (cl-transforms:3d-vector point)
      (cl-transforms:pose (cl-transforms:origin point)))
    (calculate-orientation-in-robot
     robot (cl-transforms:make-identity-rotation)))
   :tool-frame (calculate-tool
                tool-length
                (translate-grasp grasp side))
   :side side))

(defmethod pose-reachable-p ((robot robot-object) (pose cl-transforms:pose)
                             &key side (tool-frame (calculate-tool
                                                    (get-tool-length)
                                                    (cl-transforms:make-identity-rotation))))
  (declare (type robot-object robot)
           (type cl-transforms:pose pose)
           (type cl-transforms:pose tool-frame))
  (lazy-car
   (reach-pose-ik
    robot pose :tool-frame tool-frame :side side)))

(defun reach-point-ik (robot point
                        &key side (grasp :top)
                          (tool-length (get-tool-length))
                          seed-state)
  (declare (type robot-object robot)
           (type (or cl-transforms:3d-vector cl-transforms:pose) point))
  (reach-pose-ik
   robot
   (etypecase point
     (cl-transforms:3d-vector
      (cl-transforms:make-pose
       point (calculate-orientation-in-robot
              robot (cl-transforms:make-identity-rotation))))
     (cl-transforms:pose
      (cl-transforms:copy-pose
       point :orientation (calculate-orientation-in-robot
                           robot (cl-transforms:make-identity-rotation)))))
   :tool-frame (calculate-tool tool-length (translate-grasp grasp side))
   :side side
   :seed-state seed-state))

(defun reach-object-ik (robot obj
                        &key side (grasp :top)
                          (tool-length (calculate-object-tool-length obj))
                          seed-state)
  (declare (type robot-object robot)
           (type object obj))
  (reach-pose-ik
   robot (cl-transforms:copy-pose
          (pose obj) :orientation (calculate-orientation-in-robot
                                   robot (cl-transforms:make-identity-rotation)))
   :tool-frame (calculate-tool tool-length (translate-grasp grasp side))
   :side side
   :seed-state seed-state))

(defmethod reach-pose-ik ((robot robot-object) (pose cl-transforms:pose)
                          &key side tool-frame seed-state)
  (declare (type robot-object robot)
           (type cl-transforms:pose pose)
           (type (or cl-transforms:pose null) tool-frame))
  (let (;; Reference frame should rather be the torso link, not the base link
        ;; (reference-frame (cl-urdf:name (cl-urdf:root-link (slot-value robot 'urdf))))
        )
    (compute-iks
     (etypecase pose
       (pose-stamped pose)
       (cl-transforms:pose
        (pose->pose-stamped *fixed-frame* 0.0 pose)))
     :robot-state robot
     :arm side
     :tcp-in-ee-pose (or tool-frame
                      (calculate-tool
                       (get-tool-length)
                       (cl-transforms:make-identity-rotation)))
     :seed-state seed-state)))
