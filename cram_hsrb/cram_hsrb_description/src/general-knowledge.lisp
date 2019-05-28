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

(def-fact-group hsrb-metadata (robot
                               robot-odom-frame
                               robot-base-frame robot-torso-link-joint
                               arm
                               camera-frame
                               robot-pan-tilt-links robot-pan-tilt-joints)
  (<- (robot hsrb))

  (<- (robot-odom-frame hsrb "odom"))

  (<- (robot-base-frame hsrb "base_footprint"))
  (<- (robot-torso-link-joint hsrb "arm_lift_link" "arm_lift_joint"))

  ;mimic joint for arm_lift_link implemented in lower-level.lisp (cram-hsrb-projection) 
  ;(<- (robot-torso-link-joint hsrb "torso_lift_link" "torso_lift_joint"))

  (<- (arm hsrb :left))
  (<- (camera-frame hsrb "head_rgbd_sensor_link"))
  (<- (robot-pan-tilt-links hsrb "head_pan_link" "head_tilt_link"))
  (<- (robot-pan-tilt-joints hsrb "head_pan_joint" "head_tilt_joint")))

(def-fact-group location-costmap-metadata (costmap:costmap-padding
                                           costmap:costmap-manipulation-padding
                                           costmap:costmap-in-reach-distance
                                           costmap:costmap-reach-minimal-distance
                                           costmap:visibility-costmap-size)
  (<- (costmap:costmap-padding 0.3))
  (<- (costmap:costmap-manipulation-padding 0.4))
  (<- (costmap:costmap-in-reach-distance 0.9))
  (<- (costmap:costmap-reach-minimal-distance 0.2))
  (<- (costmap:visibility-costmap-size 2)))
