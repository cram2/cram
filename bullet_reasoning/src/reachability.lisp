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
                         (:right . ,(cl-transforms:euler->quaternion :az (/ pi -2)))))

(defun get-grasp (grasp side)
  (ecase grasp
    (:top (cdr (assoc :top *grasps*)))
    (:side (cdr (assoc side *grasps*)))))

(defun object-reachable-p (robot obj &key
                           side (grasp :top)
                           (tool-frame (cl-transforms:make-pose
                                        (cl-transforms:make-3d-vector 0.20 0.0 0.0)
                                        (get-grasp grasp side))))
  (lazy-car (reach-object-ik robot obj :tool-frame tool-frame :side side :grasp grasp)))

(defun reach-object-ik (robot obj &key
                        side (grasp :top)
                        (tool-frame (cl-transforms:make-pose
                                     (cl-transforms:make-3d-vector 0.20 0.0 0.0)
                                     (get-grasp grasp side))))
  (let ((obj-trans-in-robot (cl-transforms:transform*
                             (cl-transforms:transform-inv
                              (cl-transforms:reference-transform
                               (pose robot)))
                             (cl-transforms:reference-transform
                              (pose obj))))
        (reference-frame (cl-urdf:name (cl-urdf:root-link (slot-value robot 'urdf)))))
    (get-weighted-ik
     robot (tf:make-pose-stamped
            reference-frame 0.0
            (cl-transforms:translation obj-trans-in-robot)
            (cl-transforms:make-quaternion 0 0 0 1))
     :ik-namespace (ecase side
                     (:right "r_arm_ik")
                     (:left "l_arm_ik"))
     :tool-frame tool-frame
     :ik-base-link reference-frame
     :fixed-frame reference-frame
     :weights-ts '(10 10 10 0.1 0.1 0.1)
     :max-seeds 20)))
