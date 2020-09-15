;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defparameter *tcp-in-ee-pose*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector 0 0 0.3191d0)
   (cl-transforms:make-identity-rotation)))

(defparameter *standard-to-boxy-gripper-transform*
  (cl-transforms-stamped:make-identity-transform))

(def-fact-group boxy-arm-facts (arm
                                arm-joints arm-links
                                hand-links hand-link hand-finger-link
                                gripper-joint gripper-meter-to-joint-multiplier
                                gripper-minimal-position
                                gripper-convergence-delta
                                standard<-particular-gripper-transform
                                end-effector-link robot-tool-frame
                                tcp-in-ee-pose
                                robot-joint-states)

  (<- (arm :boxy-description :left))
  (<- (arm :boxy-description :right))

  (<- (arm-joints :boxy-description :left ("left_arm_0_joint"
                                           "left_arm_1_joint"
                                           "left_arm_2_joint"
                                           "left_arm_3_joint"
                                           "left_arm_4_joint"
                                           "left_arm_5_joint"
                                           "left_arm_6_joint")))
  (<- (arm-joints :boxy-description :right ("right_arm_0_joint"
                                            "right_arm_1_joint"
                                            "right_arm_2_joint"
                                            "right_arm_3_joint"
                                            "right_arm_4_joint"
                                            "right_arm_5_joint"
                                            "right_arm_6_joint")))

  (<- (arm-links :boxy-description :left ("left_arm_1_link"
                                          "left_arm_2_link"
                                          "left_arm_3_link"
                                          "left_arm_4_link"
                                          "left_arm_5_link"
                                          "left_arm_6_link"
                                          "left_arm_7_link")))
  (<- (arm-links :boxy-description :right ("right_arm_1_link"
                                           "right_arm_2_link"
                                           "right_arm_3_link"
                                           "right_arm_4_link"
                                           "right_arm_5_link"
                                           "right_arm_6_link"
                                           "right_arm_7_link")))

  (<- (hand-links :boxy-description :left (;; "left_gripper_base_link"
                                           "left_gripper_finger_left_link"
                                           "left_gripper_finger_right_link"
                                           "left_gripper_gripper_left_link"
                                           "left_gripper_gripper_right_link"
                                           "left_gripper_tool_frame")))
  (<- (hand-links :boxy-description :right (;; "right_gripper_base_link"
                                            "right_gripper_finger_left_link"
                                            "right_gripper_finger_right_link"
                                            "right_gripper_gripper_left_link"
                                            "right_gripper_gripper_right_link"
                                            "right_gripper_tool_frame")))

  (<- (hand-link :boxy-description :left ?link)
    (bound ?link)
    (lisp-fun search "left_gripper" ?link ?pos)
    (lisp-pred identity ?pos))
  (<- (hand-link :boxy-description :right ?link)
    (bound ?link)
    (lisp-fun search "right_gripper" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (hand-finger-link :boxy-description ?arm ?link)
    (bound ?link)
    (hand-link :boxy-description ?arm ?link)
    (lisp-fun search "finger" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (gripper-joint :boxy-description :left "left_gripper_joint"))
  (<- (gripper-joint :boxy-description :right "right_gripper_joint"))

  (<- (gripper-meter-to-joint-multiplier :boxy-description 1.0))
  (<- (gripper-minimal-position :boxy-description ?_ 0.0))
  (<- (gripper-convergence-delta :boxy-description ?_ 0.001))

  (<- (standard<-particular-gripper-transform :boxy-description ?transform)
    (symbol-value *standard-to-boxy-gripper-transform* ?transform))

  (<- (end-effector-link :boxy-description :left "left_arm_7_link"))
  (<- (end-effector-link :boxy-description :right "right_arm_7_link"))

  (<- (robot-tool-frame :boxy-description :left "left_gripper_tool_frame"))
  (<- (robot-tool-frame :boxy-description :right "right_gripper_tool_frame"))

  (<- (tcp-in-ee-pose :boxy-description ?pose)
    (symbol-value *tcp-in-ee-pose* ?pose))

  (<- (robot-joint-states :boxy-description :arm :left :carry
                          (("left_arm_0_joint" -1.858d0)
                           ("left_arm_1_joint" 0.70571d0)
                           ("left_arm_2_joint" 0.9614d0)
                           ("left_arm_3_joint" -0.602d0)
                           ("left_arm_4_joint" -2.5922d0)
                           ("left_arm_5_joint" -1.94065d0)
                           ("left_arm_6_joint" -1.28735d0))))
  (<- (robot-joint-states :boxy-description :arm :left :park ?joint-states)
    (robot-joint-states :boxy-description :arm :left :carry ?joint-states))
  (<- (robot-joint-states :boxy-description :arm :left :carry-top ?joint-states)
    (robot-joint-states :boxy-description :arm :left :carry ?joint-states))
  (<- (robot-joint-states :boxy-description :arm :left :carry-side-gripper-vertical
                          ?joint-states)
    (robot-joint-states :boxy-description :arm :left :carry ?joint-states))
  (<- (robot-joint-states :boxy-description :arm :left :carry-top-basket
                          ?joint-states)
    (robot-joint-states :boxy-description :arm :left :carry ?joint-states))
  (<- (robot-joint-states :boxy-description :arm :left :hand-over
                          (("left_arm_0_joint" -0.32)
                           ("left_arm_1_joint" 1.8)
                           ("left_arm_2_joint" -0.74)
                           ("left_arm_3_joint" -1.49)
                           ("left_arm_4_joint" 2.29)
                           ("left_arm_5_joint" 1.68)
                           ("left_arm_6_joint" 0.2))))

  (<- (robot-joint-states :boxy-description :arm :right :carry
                          (("right_arm_0_joint" 1.858d0)
                           ("right_arm_1_joint" -0.70571d0)
                           ("right_arm_2_joint" -0.9614d0)
                           ("right_arm_3_joint" 0.602d0)
                           ("right_arm_4_joint" 2.5922d0)
                           ("right_arm_5_joint" 1.94065d0)
                           ("right_arm_6_joint" 1.28735d0))))
  (<- (robot-joint-states :boxy-description :arm :right :park ?joint-states)
    (robot-joint-states :boxy-description :arm :right :carry ?joint-states))
  (<- (robot-joint-states :boxy-description :arm :right :carry-top ?joint-states)
    (robot-joint-states :boxy-description :arm :right :carry ?joint-states))
  (<- (robot-joint-states :boxy-description :arm :right :carry-side-gripper-vertical
                          ?joint-states)
    (robot-joint-states :boxy-description :arm :right :carry ?joint-states))

  (<- (robot-joint-states :boxy-description :arm :left :flip
                          (("left_arm_0_joint" -1.2274070978164673)
                           ("left_arm_1_joint" 0.8496202230453491)
                           ("left_arm_2_joint" -0.10349386930465698)
                           ("left_arm_3_joint" -1.0852965116500854)
                           ("left_arm_4_joint" -0.4587952196598053)
                           ("left_arm_5_joint" 1.259474515914917)
                           ("left_arm_6_joint" -0.06962397694587708)))))
