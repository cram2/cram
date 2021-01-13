;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :boxy-descr)

(defparameter *neck-ee-p-camera*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector
    0.09862691563322334d0 0.03128412196559971d0 0.04354402436024185d0)
   (cl-transforms:make-quaternion
    -0.7013586107397135d0 -0.003984622804686699d0 0.0013402975558304106d0
    0.7127960729748405d0)))


(def-fact-group boxy-neck-facts (camera-frame
                                 camera-in-neck-ee-pose
                                 neck-camera-z-offset
                                 neck-camera-pose-unit-vector-multiplier
                                 neck-camera-resampling-step
                                 neck-camera-x-axis-limit neck-camera-y-axis-limit
                                 neck-camera-z-axis-limit
                                 camera-horizontal-angle camera-vertical-angle
                                 camera-minimal-height camera-maximal-height
                                 robot-neck-links
                                 robot-neck-joints
                                 robot-neck-base-link
                                 robot-joint-states)

  (<- (camera-frame :boxy-description "head_mount_kinect2_rgb_optical_frame"))

  (<- (camera-in-neck-ee-pose :boxy-description ?pose)
    (symbol-value *neck-ee-p-camera* ?pose))

  (<- (neck-camera-z-offset :boxy-description 0.1))
  (<- (neck-camera-pose-unit-vector-multiplier :boxy-description 0.4))
  (<- (neck-camera-resampling-step :boxy-description 0.1))
  (<- (neck-camera-x-axis-limit :boxy-description 0.2))
  (<- (neck-camera-y-axis-limit :boxy-description 0.2))
  (<- (neck-camera-z-axis-limit :boxy-description 0.2))

  ;; These are values taken from the Kinect's wikipedia page for the 360 variant
  (<- (camera-horizontal-angle :boxy-description 0.99483)) ;  ca 57 degrees
  (<- (camera-vertical-angle :boxy-description 0.75049))   ; ca 43 degrees

  ;; (<- (camera-minimal-height :boxy-description 1.0))
  ;; (<- (camera-maximal-height :boxy-description 2.5))

  (<- (robot-neck-links :boxy-description
                        "neck_shoulder_link"
                        "neck_upper_arm_link"
                        "neck_forearm_link"
                        "neck_wrist_1_link"
                        "neck_wrist_2_link"
                        "neck_wrist_3_link"))

  (<- (robot-neck-joints :boxy-description
                         "neck_shoulder_pan_joint"
                         "neck_shoulder_lift_joint"
                         "neck_elbow_joint"
                         "neck_wrist_1_joint"
                         "neck_wrist_2_joint"
                         "neck_wrist_3_joint"))

  (<- (robot-neck-base-link :boxy-description "neck_base_link"))

  (<- (robot-joint-states :boxy-description :neck ?there-is-only-one-neck :away
                          (("neck_shoulder_pan_joint" -1.3155d0)
                           ("neck_shoulder_lift_joint" -1.181355d0)
                           ("neck_elbow_joint" -1.9562d0)
                           ("neck_wrist_1_joint" 0.142417d0)
                           ("neck_wrist_2_joint" 1.13492d0)
                           ("neck_wrist_3_joint" 0.143467d0))))
  (<- (robot-joint-states :boxy-description :neck ?there-is-only-one-neck :forward
                          ?joint-states)
    (robot-joint-states :boxy-description :neck ?there-is-only-one-neck :away
                        ?joint-states))
  (<- (robot-joint-states :boxy-description :neck ?there-is-only-one-neck :down
                          (("neck_shoulder_pan_joint" -1.176d0)
                           ("neck_shoulder_lift_joint" -3.1252d0)
                           ("neck_elbow_joint" -0.8397d0)
                           ("neck_wrist_1_joint" 0.83967d0)
                           ("neck_wrist_2_joint" 1.1347d0)
                           ("neck_wrist_3_joint" -0.0266d0))))
  (<- (robot-joint-states :boxy-description :neck ?only-one-neck :down-left
                          (("neck_shoulder_pan_joint" -0.776d0)
                           ("neck_shoulder_lift_joint" -3.1252d0)
                           ("neck_elbow_joint" -0.8397d0)
                           ("neck_wrist_1_joint" 0.83967d0)
                           ("neck_wrist_2_joint" 1.1347d0)
                           ("neck_wrist_3_joint" -0.0266d0)))))
