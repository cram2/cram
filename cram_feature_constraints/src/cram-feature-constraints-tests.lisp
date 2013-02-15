;;;
;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :cram-feature-constraints)

(defun hold-left-arm-before-chest ()
  (let* ((chest-plane (make-instance
                       'geometric-feature
                       :name "plane in front of chest"
                       :frame-id "/torso_lift_link"
                       :feature-type 'plane
                       :feature-position (cl-transforms:make-3d-vector 0.2 0.0 0.0)
                       :feature-direction (cl-transforms:make-3d-vector 1.0 0.0 0.0)
                       :contact-direction (cl-transforms:make-3d-vector 0.0 1.0 0.0)))
         (belly-plane (make-instance
                       'geometric-feature
                       :name "plane through belly"
                       :frame-id "/torso_lift_link"
                       :feature-type 'plane
                       :feature-position (cl-transforms:make-3d-vector 0.2 0.0 -0.1)
                       :feature-direction (cl-transforms:make-3d-vector 0.0 0.0 1.0)
                       :contact-direction (cl-transforms:make-3d-vector 1.0 0.0 0.0)))
         (center-chest-plane (make-instance
                       'geometric-feature
                       :name "plane through center of chest"
                       :frame-id "/torso_lift_link"
                       :feature-type 'plane
                       :feature-position (cl-transforms:make-3d-vector 0.2 0.0 0.0)
                       :feature-direction (cl-transforms:make-3d-vector 0.0 1.0 0.0)
                       :contact-direction (cl-transforms:make-3d-vector 1.0 0.0 0.0)))
         (left-gripper-plane 
           (make-instance
            'geometric-feature
            :name "left gripper approximated by plane"
            :frame-id "/l_gripper_tool_frame"
            :feature-type 'plane
            :feature-position (cl-transforms:make-3d-vector 0.0 0.0 0.0)
            :feature-direction (cl-transforms:make-3d-vector 0.0 0.0 0.1)
            :contact-direction (cl-transforms:make-3d-vector 1.0 0.0 0.0)))
         (in-front-chest-constraint
           (make-instance
            'feature-constraint
            :name "left gripper in front of chest"
            :feature-function "height"
            :tool-feature left-gripper-plane
            :world-feature chest-plane
            :lower-boundary 0.1
            :upper-boundary 0.3
            :weight 1.0
            :maximum-velocity 0.3
            :minimum-velocity -0.3))
         (left-body-side-constraint
           (make-instance
            'feature-constraint
            :name "left body side constraint"
            :feature-function "height"
            :tool-feature left-gripper-plane
            :world-feature center-chest-plane
            :lower-boundary 0.0
            :upper-boundary 0.25
            :weight 1.0
            :maximum-velocity 0.3
            :minimum-velocity -0.3))
         (above-belly-constraint
           (make-instance
            'feature-constraint
            :name "above belly constraint"
            :feature-function "height"
            :tool-feature left-gripper-plane
            :world-feature belly-plane
            :lower-boundary 0.18
            :upper-boundary 0.22
            :weight 1.0
            :maximum-velocity 0.3
            :minimum-velocity -0.3))
         (gripper-vertical-constraint
           (make-instance
            'feature-constraint
            :name "gripper vertical constraint"
            :feature-function "perpendicular"
            :tool-feature left-gripper-plane
            :world-feature belly-plane
            :lower-boundary 0.98
            :upper-boundary 1.0
            :weight 1.0
            :maximum-velocity 0.3
            :minimum-velocity -0.3)))
    (list
     in-front-chest-constraint
     left-body-side-constraint
     above-belly-constraint
     gripper-vertical-constraint)))