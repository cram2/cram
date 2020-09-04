;;; Copyright (c) 2015, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-robot-interfaces)

(def-fact-group neck (camera-frame
                      camera-minimal-height camera-maximal-height
                      camera-horizontal-angle camera-vertical-angle
                      neck robot-neck-links robot-neck-joints robot-neck-base-link
                      camera-in-neck-ee-pose
                      neck-camera-z-offset
                      neck-camera-pose-unit-vector-multiplier
                      neck-camera-resampling-step
                      neck-camera-x-axis-limit neck-camera-y-axis-limit
                      neck-camera-z-axis-limit)

  ;; Unifies ?frame with the name of the camera frame present on the ?robot
  (<- (camera-frame ?robot ?frame)
    (fail))

  ;; ?min-height is possible minimal distance (in meters) of the camera from
  ;; the 'ground'.
  ;; E.g. for PR2 it's the Z pose when the torso is maximally down.
  ;; For a quadrotor it will be close to 0, as it can stand on the ground
  (<- (camera-minimal-height ?robot ?min-height)
    (fail))
  ;; ?max-height is the opposite of ?min-height, the maximal possible
  ;; distance from the ground. For quadrotors it will be the maximal
  ;; distance it can fly off the ground...
  (<- (camera-maximal-height ?robot ?max-height)
    (fail))

  ;; view angle of the camera (assuming a fixed focal length) in horizontal axis
  (<- (camera-horizontal-angle ?robot ?angle)
    (fail))
  ;; view angle of the camera (assuming a fixed focal length) in vertical axis
  (<- (camera-vertical-angle ?robot ?angle)
    (fail))

  ;; Unifies ?neck with the name of a body part that is present on the ?robot.
  (<- (neck ?robot ?neck)
    (fail))

  ;; For necks with standard pan-tilt units (2 joints pan and tilt)
  (<- (robot-neck-links ?robot ?pan-link ?tilt-link)
    (fail))
  (<- (robot-neck-joints ?robot ?pan-joint ?tilt-joint)
    (fail))

  ;; For necks with more than 2 joints
  (<- (robot-neck-links ?robot . ?neck-links)
    (fail))
  (<- (robot-neck-joints ?robot . ?neck-joints)
    (fail))
  ;; This link will be used for IK stuff (if IK will be used)
  (<- (robot-neck-base-link ?robot ?neck-base-link)
    (fail))
  ;; neck_P_camera cl-transforms:pose
  (<- (camera-in-neck-ee-pose ?robot ?pose)
    (fail))
  ;; for doing neck IK if the neck has more than 2 joints
  (<- (neck-camera-z-offset ?robot ?number)
    (fail))
  (<- (neck-camera-pose-unit-vector-multiplier ?robot ?number)
    (fail))
  (<- (neck-camera-resampling-step ?robot ?number)
    (fail))
  (<- (neck-camera-x-axis-limit ?robot ?number)
    (fail))
  (<- (neck-camera-y-axis-limit ?robot ?number)
    (fail))
  (<- (neck-camera-z-axis-limit ?robot ?number)
    (fail)))
