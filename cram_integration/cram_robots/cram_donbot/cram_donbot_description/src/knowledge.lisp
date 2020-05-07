;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :donbot-descr)

(defparameter *tcp-in-ee-pose*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector 0 0 0.2581d0)
   (cl-transforms:make-quaternion 0 0 -0.7071067811849626d0 -0.7071067811881324d0)))

(defparameter *standard-to-donbot-gripper-transform*
  (cl-transforms-stamped:make-identity-transform))

(defparameter *parking-joint-state*
  '(("ur5_shoulder_pan_joint" 3.234022855758667d0)
    ("ur5_shoulder_lift_joint" -1.5068710486041468d0)
    ("ur5_elbow_joint" -0.7870314756976526d0)
    ("ur5_wrist_1_joint" -2.337625328694479d0)
    ("ur5_wrist_2_joint" 1.5699548721313477d0)
    ("ur5_wrist_3_joint" -1.6504042784320276d0)))
;; (defparameter *parking-joint-state*
;;   '(("ur5_shoulder_pan_joint" 3.378162384033203d0)
;;     ("ur5_shoulder_lift_joint" -1.5641868750201624d0)
;;     ("ur5_elbow_joint" -0.9430778662310999d0)
;;     ("ur5_wrist_1_joint" -2.2492716948138636d0)
;;     ("ur5_wrist_2_joint" 1.5673996210098267d0)
;;     ("ur5_wrist_3_joint" -3.5105767885791224d0)))

(defparameter *looking-down-state*
  '(("ur5_shoulder_pan_joint" 4.130944728851318d0)
    ("ur5_shoulder_lift_joint" 0.04936718940734863d0)
    ("ur5_elbow_joint" -1.9734209219561976d0)
    ("ur5_wrist_1_joint" -1.7624157110797327d0)
    ("ur5_wrist_2_joint" 1.6369260549545288d0)
    ("ur5_wrist_3_joint" -1.6503327528582972d0)))

;; (defparameter *looking-forward-state*
;;   '(("ur5_shoulder_pan_joint" 0.29503026604652405d0)
;;     ("ur5_shoulder_lift_joint" -1.8346226851092737d0)
;;     ("ur5_elbow_joint" 2.1440072059631348d0)
;;     ("ur5_wrist_1_joint" -3.4500272909747522d0)
;;     ("ur5_wrist_2_joint" -1.8935192267047327d0)
;;     ("ur5_wrist_3_joint" -1.425162140523092d0)))

(defparameter *looking-right-state*
  '(("ur5_shoulder_pan_joint" 1.6281344890594482d0)
    ("ur5_shoulder_lift_joint" -1.4734271208392542d0)
    ("ur5_elbow_joint" -1.1555221716510218d0)
    ("ur5_wrist_1_joint" -0.9881671110736292d0)
    ("ur5_wrist_2_joint" 1.4996352195739746d0)
    ("ur5_wrist_3_joint" -1.4276765028582972d0)))

(defparameter *looking-right-separators-state*
  '(("ur5_shoulder_pan_joint" 1.4752850532531738d0)
    ("ur5_shoulder_lift_joint" -1.4380276838885706d0)
    ("ur5_elbow_joint" -1.9198325316058558d0)
    ("ur5_wrist_1_joint" -0.0680769125567835d0)
    ("ur5_wrist_2_joint" 1.704722285270691d0)
    ("ur5_wrist_3_joint" -1.5686963240252894d0)))

(defparameter *looking-right-separators-preplace-state*
  '(("ur5_shoulder_pan_joint" 1.585883378982544d0)
    ("ur5_shoulder_lift_joint" -0.4995768705951136d0)
    ("ur5_elbow_joint" -1.61414081255068d0)
    ("ur5_wrist_1_joint" -1.1720898787127894d0)
    ("ur5_wrist_2_joint" 1.37771737575531d0)
    ("ur5_wrist_3_joint" -1.3602331320392054d0)))

(defparameter *ee-p-camera*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector -0.04214122915220634d0
                                 0.022506720839268142d0
                                 0.1335101519901074d0)
   (cl-transforms:make-quaternion -0.006114286700067816d0
                                  0.003707211262453658d0
                                  -0.7139449315234562d0
                                  0.7001653424975933d0)))



(def-fact-group donbot-metadata (robot
                                 robot-odom-frame
                                 robot-base-frame robot-torso-link-joint
                                 arm
                                 camera-frame)
  (<- (robot iai-donbot))

  (<- (robot-odom-frame iai-donbot "odom"))

  (<- (robot-base-frame iai-donbot "base_footprint"))
  (<- (robot-torso-link-joint iai-donbot "ur5_base_link" "arm_base_mounting_joint"))

  (<- (arm iai-donbot :left))

  (<- (camera-frame iai-donbot "camera_link")) ; rgb camera for barcodes etc.
  (<- (camera-frame iai-donbot "rs_camera_depth_optical_frame")) ; realsense
  (<- (camera-frame iai-donbot "rs_camera_color_optical_frame")))



(def-fact-group donbot-arm-facts (end-effector-link
                                  robot-tool-frame
                                  arm-joints arm-links
                                  gripper-joint gripper-link
                                  gripper-meter-to-joint-multiplier
                                  standard-to-particular-gripper-transform
                                  robot-joint-states
                                  tcp-in-ee-pose
                                  hand-links)

  (<- (end-effector-link iai-donbot :left "ur5_wrist_3_link"))

  (<- (robot-tool-frame iai-donbot :left "refills_tool_frame"))

  (<- (arm-joints iai-donbot :left ("ur5_shoulder_pan_joint"
                                "ur5_shoulder_lift_joint"
                                "ur5_elbow_joint"
                                "ur5_wrist_1_joint"
                                "ur5_wrist_2_joint"
                                "ur5_wrist_3_joint")))

  (<- (arm-links iai-donbot :left ("ur5_shoulder_link"
                               "ur5_upper_arm_link"
                               "ur5_forearm_link"
                               "ur5_wrist_1_link"
                               "ur5_wrist_2_link"
                               "ur5_wrist_3_link")))

  (<- (gripper-joint iai-donbot :left "gripper_joint"))

  (<- (gripper-link iai-donbot :left ?link)
    (bound ?link)
    (lisp-fun search "gripper_" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (gripper-meter-to-joint-multiplier iai-donbot 1.0))

  (<- (standard-to-particular-gripper-transform iai-donbot ?transform)
    (symbol-value *standard-to-donbot-gripper-transform* ?transform))

  (<- (robot-joint-states iai-donbot :arm :left :carry ?joint-states)
    (symbol-value ;; *looking-forward-state*
     *parking-joint-state*
     ?joint-states))

  (<- (robot-joint-states iai-donbot :arm :left :park ?joint-states)
    (symbol-value *parking-joint-state* ?joint-states))

  (<- (tcp-in-ee-pose iai-donbot ?pose)
    (symbol-value *tcp-in-ee-pose* ?pose))

  (<- (hand-links iai-donbot :left ("wrist_collision"
                                "gripper_base_link"
                                "gripper_finger_left_link"
                                "gripper_finger_right_link"
                                "gripper_gripper_left_link"
                                "gripper_gripper_right_link"))))



(def-fact-group donbot-neck-facts (robot-neck-links
                                   robot-neck-joints
                                   robot-neck-base-link
                                   robot-joint-states
                                   camera-in-neck-ee-pose)

  (<- (robot-neck-links iai-donbot . ?links)
    (arm-links iai-donbot :left ?links))

  (<- (robot-neck-joints iai-donbot . ?joints)
    (arm-joints iai-donbot :left ?joints))

  (<- (robot-neck-base-link iai-donbot "ur5_base_link"))

  (<- (robot-joint-states iai-donbot :neck ?there-is-only-one-neck :away ?joint-states)
    (symbol-value *parking-joint-state* ?joint-states))

  ;; (<- (robot-joint-states iai-donbot :neck ?there-is-only-one-neck :forward ?joint-states)
  ;;   (symbol-value *looking-forward-state* ?joint-states))

  (<- (robot-joint-states iai-donbot :neck ?there-is-only-one-neck :down ?joint-states)
    (symbol-value *looking-down-state* ?joint-states))

  (<- (robot-joint-states iai-donbot :neck ?there-is-only-one-neck :right ?joint-states)
    (symbol-value *looking-right-state* ?joint-states))

  (<- (robot-joint-states iai-donbot :neck ?there-is-only-one-neck :right-separators
                          ?joint-states)
    (symbol-value *looking-right-separators-state* ?joint-states))

  (<- (robot-joint-states iai-donbot :neck ?there-is-only-one-neck :right-separators-preplace-state
                          ?joint-states)
    (symbol-value *looking-right-separators-preplace-state* ?joint-states))

  (<- (camera-in-neck-ee-pose iai-donbot ?pose)
    (symbol-value *ee-p-camera* ?pose)))


(def-fact-group location-costmap-metadata (costmap:costmap-padding
                                           costmap:costmap-manipulation-padding
                                           costmap:costmap-in-reach-distance
                                           costmap:costmap-reach-minimal-distance
                                           costmap:visibility-costmap-size)
  (<- (costmap:costmap-padding 0.5))
  (<- (costmap:costmap-manipulation-padding 0.5))
  (<- (costmap:costmap-in-reach-distance 1.2))
  (<- (costmap:costmap-reach-minimal-distance 0.2))
  (<- (costmap:visibility-costmap-size 2)))
