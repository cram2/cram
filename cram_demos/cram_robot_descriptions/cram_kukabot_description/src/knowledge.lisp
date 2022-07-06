;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :kukabot-descr)

(defparameter *tcp-in-ee-pose*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector 0 0 0.2291)
   (cl-transforms:make-quaternion 0 0 1 0)))

(defparameter *standard-to-donbot-gripper-transform*
  (cl-transforms-stamped:make-identity-transform))

(defparameter *ee-p-camera*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector -0.001328999336864145d0
                                 0.06619000048451928d0
                                 0.11668299973270257d0)
   (cl-transforms:make-quaternion 0.11535978142567395d0
                                  -0.0013328246250186582d0
                                  0.0036558132247416775d0
                                  0.9933161528121878d0)))


(def-fact-group kukabot-metadata (robot-odom-frame
                                 robot-base-frame robot-base-link
                                 robot-torso-link-joint
                                 arm neck
                                 camera-frame
                                 camera-minimal-height camera-maximal-height
                                 camera-horizontal-angle camera-vertical-angle)

  (<- (robot-odom-frame :kmr-iiwa "odom"))

  (<- (robot-base-frame :kmr-iiwa "base_link"))
  (<- (robot-base-link :kmr-iiwa "base_link"))
  (<- (robot-torso-link-joint :kmr-iiwa "angle_adapter_plate" "angle_adapter_plate_joint"))

  (<- (arm :kmr-iiwa :left))

  (<- (neck :kmr-iiwa :left))

  (<- (camera-frame :kmr-iiwa "camera_link")) ; rgb camera for barcodes etc.
  ;; (<- (camera-frame :kmr-iiwa "rs_camera_depth_optical_frame")) ; realsense, virtual
  ;; (<- (camera-frame :kmr-iiwa "rs_camera_color_optical_frame"))
                                        ; virtual

  ;; The visibility costmap makes no sense for Kukabot, as it can move its arm anywhere.
  ;; (<- (camera-minimal-height :kmr-iiwa 0.7))
  ;; (<- (camera-maximal-height :kmr-iiwa 2.0))

  ;; These are values taken from the Kinect's wikipedia page for the 360 variant
  (<- (camera-horizontal-angle :kmr-iiwa 0.99483))
  (<- (camera-vertical-angle :kmr-iiwa 0.75049)))


(def-fact-group kukabot-arm-facts (arm-joints
                                   arm-links
                                   hand-links hand-link hand-finger-link
                                   gripper-joint
                                   gripper-meter-to-joint-multiplier
                                   gripper-minimal-position
                                   gripper-convergence-delta
                                   standard<-particular-gripper-transform
                                   end-effector-link
                                   robot-tool-frame
                                   tcp-in-ee-pose
                                   robot-joint-states)

  (<- (arm-joints :kmr-iiwa :left ("iiwa_joint_1"
                                   "iiwa_joint_2"
                                   "iiwa_joint_3"
                                   "iiwa_joint_4"
                                   "iiwa_joint_5"
                                   "iiwa_joint_6"
                                   "iiwa_joint_7")))

  (<- (arm-links :kmr-iiwa :left ("iiwa_link_1"
                                  "iiwa_link_2"
                                  "iiwa_link_3"
                                  "iiwa_link_4"
                                  "iiwa_link_5"
                                  "iiwa_link_6"
                                  "iiwa_link_7")))

  (<- (hand-links :kmr-iiwa :left ("wrist_collision"
                                   "fwk_fwa_050_frame_in"
                                   "camera_holder_link"
                                   "gripper_base_link"
                                   "gripper_finger_left_link"
                                   "gripper_finger_right_link")))

  (<- (hand-link :kmr-iiwa :left ?link)
    (bound ?link)
    (or (and (lisp-fun search "gripper_" ?link ?pos)
             (lisp-pred identity ?pos))
        (and (lisp-fun search "finger" ?link ?pos)
             (lisp-pred identity ?pos))))

  (<- (hand-finger-link :kmr-iiwa ?arm ?link)
    (bound ?link)
    (hand-link :kmr-iiwa ?arm ?link)
    (lisp-fun search "finger" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (gripper-joint :kmr-iiwa :left "gripper_joint"))

  (<- (gripper-meter-to-joint-multiplier :kmr-iiwa 1.0))
  (<- (gripper-minimal-position :kmr-iiwa ?_ 0.0))
  (<- (gripper-convergence-delta :kmr-iiwa ?_ 0.001))

  (<- (standard<-particular-gripper-transform :kmr-iiwa ?transform)
    (symbol-value *standard-to-donbot-gripper-transform* ?transform))

  (<- (end-effector-link :kmr-iiwa :left "iiwa_link_7"))

  (<- (robot-tool-frame :kmr-iiwa :left "gripper_tool_frame"))

  (<- (tcp-in-ee-pose :kmr-iiwa ?pose)
    (symbol-value *tcp-in-ee-pose* ?pose))

  (<- (robot-joint-states :kmr-iiwa :arm :left :carry
                          (("iiwa_joint_1" 0.0)
                           ("iiwa_joint_2" -1.28)
                           ("iiwa_joint_3" 0.0)
                           ("iiwa_joint_4" 1.29)
                           ("iiwa_joint_5" 0.0)
                           ("iiwa_joint_6" -1.0)
                           ("iiwa_joint_7" 1.57))))
  (<- (robot-joint-states :kmr-iiwa :arm :left :park
                          (("iiwa_joint_1" 0.0)
                           ("iiwa_joint_2" -1.23)
                           ("iiwa_joint_3" 0.0)
                           ("iiwa_joint_4" 1.38999978643)
                           ("iiwa_joint_5" 0.0)
                           ("iiwa_joint_6" -0.85)
                           ("iiwa_joint_7" 0.0))))
  (<- (robot-joint-states :kmr-iiwa :arm :left :carry-top ?joint-states)
    (robot-joint-states :kmr-iiwa :arm :left :carry ?joint-states))
  (<- (robot-joint-states :kmr-iiwa :arm :left :carry-side-gripper-vertical
                          ?joint-states)
    (robot-joint-states :kmr-iiwa :arm :left :carry ?joint-states)))



(def-fact-group kukabot-neck-facts (robot-neck-links
                                    robot-neck-joints
                                    robot-neck-base-link
                                    camera-in-neck-ee-pose
                                    neck-camera-z-offset
                                    neck-camera-pose-unit-vector-multiplier
                                    neck-camera-resampling-step
                                    neck-camera-x-axis-limit
                                    neck-camera-y-axis-limit
                                    neck-camera-z-axis-limit
                                    robot-joint-states)

  (<- (robot-neck-links :kmr-iiwa . ?links)
    (arm-links :kmr-iiwa :left ?links))

  (<- (robot-neck-joints :kmr-iiwa . ?joints)
    (arm-joints :kmr-iiwa :left ?joints))

  (<- (robot-neck-base-link :kmr-iiwa "iiwa_link_0"))

  (<- (camera-in-neck-ee-pose :kmr-iiwa ?pose)
    (symbol-value *ee-p-camera* ?pose))

  (<- (neck-camera-z-offset :kmr-iiwa 0.7))
  (<- (neck-camera-pose-unit-vector-multiplier :kmr-iiwa 0.5))
  (<- (neck-camera-resampling-step :kmr-iiwa 0.2))
  (<- (neck-camera-x-axis-limit :kmr-iiwa 0.5))
  (<- (neck-camera-y-axis-limit :kmr-iiwa 0.5))
  (<- (neck-camera-z-axis-limit :kmr-iiwa 0.2))

  (<- (robot-joint-states :kmr-iiwa :neck ?there-is-only-one-neck :away
                          (("iiwa_joint_1" 0.0)
                           ("iiwa_joint_2" -1.23)
                           ("iiwa_joint_3" 0.0)
                           ("iiwa_joint_4" 1.38999978643)
                           ("iiwa_joint_5" 0.0)
                           ("iiwa_joint_6" -0.85)
                           ("iiwa_joint_7" 0.0))))
  (<- (robot-joint-states :kmr-iiwa :neck ?there-is-only-one-neck :forward
                          (("iiwa_joint_1" 0.87564548888)
                           ("iiwa_joint_2" -1.18868177242)
                           ("iiwa_joint_3" -0.611353182624)
                           ("iiwa_joint_4" 1.53584067968)
                           ("iiwa_joint_5" 1.81510903672)
                           ("iiwa_joint_6" -1.63964857509)
                           ("iiwa_joint_7" -0.16138790158))))
  (<- (robot-joint-states :kmr-iiwa :neck ?there-is-only-one-neck :down
                          (("iiwa_joint_1" 0.0)
                           ("iiwa_joint_2" -1.23)
                           ("iiwa_joint_3" 0.0)
                           ("iiwa_joint_4" 1.38999978643)
                           ("iiwa_joint_5" 0.0)
                           ("iiwa_joint_6" -0.85)
                           ("iiwa_joint_7" 0.0))))
  (<- (robot-joint-states :kmr-iiwa :neck ?there-is-only-one-neck :right
                          (("iiwa_joint_1" 0.0)
                           ("iiwa_joint_2" -1.23)
                           ("iiwa_joint_3" 0.0)
                           ("iiwa_joint_4" 1.38999978643)
                           ("iiwa_joint_5" 0.0)
                           ("iiwa_joint_6" -0.85)
                           ("iiwa_joint_7" 0.0))))
  (<- (robot-joint-states :kmr-iiwa :neck ?there-is-only-one-neck
                                    :right-separators
                                    (("iiwa_joint_1" 0.0)
                                     ("iiwa_joint_2" -1.23)
                                     ("iiwa_joint_3" 0.0)
                                     ("iiwa_joint_4" 1.38999978643)
                                     ("iiwa_joint_5" 0.0)
                                     ("iiwa_joint_6" -0.85)
                                     ("iiwa_joint_7" 0.0))))
  (<- (robot-joint-states :kmr-iiwa :neck ?there-is-only-one-neck
                                    :right-separators-preplace-state
                                    (("iiwa_joint_1" 0.0)
                                     ("iiwa_joint_2" -1.23)
                                     ("iiwa_joint_3" 0.0)
                                     ("iiwa_joint_4" 1.38999978643)
                                     ("iiwa_joint_5" 0.0)
                                     ("iiwa_joint_6" -0.85)
                                     ("iiwa_joint_7" 0.0)))))


(def-fact-group location-costmap-metadata (costmap:costmap-padding
                                           costmap:costmap-manipulation-padding
                                           costmap:costmap-in-reach-distance
                                           costmap:costmap-reach-minimal-distance
                                           costmap:orientation-samples
                                           costmap:orientation-sample-step
                                           costmap:reachability-orientation-offset
                                           costmap:visibility-orientation-offset
                                           costmap:visibility-costmap-size)
  (<- (costmap:costmap-padding :kmr-iiwa 0.5))
  (<- (costmap:costmap-manipulation-padding :kmr-iiwa 0.3))
  (<- (costmap:costmap-in-reach-distance :kmr-iiwa 1.4))
  (<- (costmap:costmap-reach-minimal-distance :kmr-iiwa 0.1))
  (<- (costmap:orientation-samples :kmr-iiwa 3))
  (<- (costmap:orientation-sample-step :kmr-iiwa 0.7854))
  (<- (costmap:reachability-orientation-offset :kmr-iiwa 0.0))
  (<- (costmap:visibility-orientation-offset :kmr-iiwa ?offset)
    (costmap:reachability-orientation-offset :kmr-iiwa ?offset))
  (<- (costmap:visibility-costmap-size :kmr-iiwa 2.0)))
