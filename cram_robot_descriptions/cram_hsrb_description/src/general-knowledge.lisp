;;;
;;; Copyright (c) 2019, Vanessa Hassouna <hassouna@uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :hsrb-descr)

(defparameter *forward-looking-position-in-base-frame*
  (cl-transforms:make-3d-vector 10.0 0.0 1.5))

(def-fact-group hsrb-metadata (robot-odom-frame
                               robot-base-frame robot-base-link
                               robot-torso-link-joint
                               arm
                               camera-frame
                               camera-horizontal-angle camera-vertical-angle
                               robot-neck-links robot-neck-joints
                               robot-neck-pan-joint-forward-facing-axis-sign
                               robot-neck-tilt-joint-forward-facing-axis-sign
                               robot-joint-states robot-pose)

  (<- (robot-odom-frame :hsrb "odom"))
  (<- (robot-base-frame :hsrb "base_footprint"))
  (<- (robot-base-link :hsrb "base_link"))
  (<- (robot-torso-link-joint :hsrb "arm_lift_link" "arm_lift_joint"))

  (<- (camera-frame :hsrb "head_center_camera_frame"))
  (<- (camera-frame :hsrb "head_rgbd_sensor_link"))

  ;; These are values taken from the Kinect's wikipedia page for the 360 variant
  (<- (camera-horizontal-angle :hsrb 0.99483))
  (<- (camera-vertical-angle :hsrb 0.75049))

  (<- (robot-neck-links :hsrb "head_pan_link" "head_tilt_link"))
  (<- (robot-neck-joints :hsrb "head_pan_joint" "head_tilt_joint"))

  ;; TODO: this needs to be corrected for this robot:
  (<- (robot-neck-pan-joint-forward-facing-axis-sign :hsrb
                                                     cl-transforms:x +1))
  (<- (robot-neck-tilt-joint-forward-facing-axis-sign :hsrb
                                                      cl-transforms:x +1))

  (<- (robot-joint-states :hsrb :neck ?_ :forward ((?pan_joint 0.0) (?tilt_joint 0.0)))
    (robot-neck-joints :hsrb ?pan_joint ?tilt_joint))

  (<- (robot-pose :hsrb :neck ?_ :forward ?pose-stamped)
    (robot-base-frame :hsrb ?base-frame)
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
  (<- (costmap:costmap-padding :hsrb 0.2))
  (<- (costmap:costmap-manipulation-padding :hsrb 0.3))
  (<- (costmap:costmap-in-reach-distance :hsrb 0.5))
  (<- (costmap:costmap-reach-minimal-distance :hsrb 0.2))
  (<- (costmap:orientation-samples :hsrb 3))
  (<- (costmap:orientation-sample-step :hsrb 0.3))
  (<- (costmap:visibility-costmap-size :hsrb 2)))
