;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-pr2-description)

;; (def-tool (cl-transforms:make-3d-vector 1 0 0) 0.20)

(defparameter *forward-looking-position-in-base-frame*
  (cl-transforms:make-3d-vector 10.0 0.0 1.5))

(def-fact-group pr2-metadata (robot
                              robot-base-frame robot-torso-link-joint
                              robot-odom-frame
                              camera-frame camera-minimal-height camera-maximal-height
                              camera-horizontal-angle camera-vertical-angle
                              robot-neck-links robot-neck-joints
                              robot-joint-states robot-pose)
  (<- (robot pr2))

  (<- (robot-odom-frame pr2 "odom_combined"))

  (<- (robot-base-frame pr2 "base_footprint"))
  (<- (robot-torso-link-joint pr2 "torso_lift_link" "torso_lift_joint"))

  (<- (camera-frame pr2 "head_mount_kinect_rgb_optical_frame"))
  (<- (camera-frame pr2 "openni_rgb_optical_frame"))
  (<- (camera-frame pr2 "narrow_stereo_optical_frame"))

  (<- (camera-minimal-height pr2 1.27))
  (<- (camera-maximal-height pr2 1.60))

  ;; These are values taken from the Kinect's wikipedia page for the 360 variant
  (<- (camera-horizontal-angle pr2 0.99483)) ;  ca 57 degrees
  (<- (camera-vertical-angle pr2 0.75049))   ; ca 43 degrees

  (<- (robot-neck-links pr2 "head_pan_link" "head_tilt_link"))
  (<- (robot-neck-joints pr2 "head_pan_joint" "head_tilt_joint"))

  (<- (robot-joint-states pr2 :neck ?_ :forward ((?pan_joint 0.0) (?tilt_joint 0.0)))
    (robot-neck-joints pr2 ?pan_joint ?tilt_joint))

  (<- (robot-pose pr2 :neck ?_ :forward ?pose-stamped)
    (robot-base-frame pr2 ?base-frame)
    (lisp-fun cl-transforms:make-identity-rotation ?identity-quaternion)
    (symbol-value *forward-looking-position-in-base-frame* ?forward-point)
    (lisp-fun cl-transforms-stamped:make-pose-stamped
              ?base-frame 0.0 ?forward-point ?identity-quaternion
              ?pose-stamped)))

(def-fact-group location-costmap-metadata (costmap:costmap-padding
                                           costmap:costmap-manipulation-padding
                                           costmap:costmap-in-reach-distance
                                           costmap:costmap-reach-minimal-distance
                                           costmap:orientation-samples
                                           costmap:orientation-sample-step
                                           costmap:visibility-costmap-size)
  (<- (costmap:costmap-padding 0.3))
  (<- (costmap:costmap-manipulation-padding 0.4))
  (<- (costmap:costmap-in-reach-distance 1.0))
  (<- (costmap:costmap-reach-minimal-distance 0.2))
  (<- (costmap:orientation-samples 1))
  (<- (costmap:orientation-sample-step 0.3))
  (<- (costmap:visibility-costmap-size 2)))
