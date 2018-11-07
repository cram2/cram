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

(defparameter *standard-to-boxy-gripper-transform*
  (cl-transforms-stamped:make-identity-transform))

(defparameter *left-parking-joint-states*
  '(("left_arm_0_joint" -1.858d0)
    ("left_arm_1_joint" 0.70571d0)
    ("left_arm_2_joint" 0.9614d0)
    ("left_arm_3_joint" -0.602d0)
    ("left_arm_4_joint" -2.5922d0)
    ("left_arm_5_joint" -1.94065d0)
    ("left_arm_6_joint" -1.28735d0)))

(defparameter *right-parking-joint-states*
  '(("right_arm_0_joint" 0.0)
    ("right_arm_1_joint" 0.0)
    ("right_arm_2_joint" 0.0)
    ("right_arm_3_joint" 0.0)
    ("right_arm_4_joint" 0.0)
    ("right_arm_5_joint" 0.0)
    ("right_arm_6_joint" 0.0)))

(defparameter *left-arm-nicer-configuration*
  '(-1.2274070978164673
    0.8496202230453491
    -0.10349386930465698
    -1.0852965116500854
    -0.4587952196598053
    1.259474515914917
    -0.06962397694587708))

(def-fact-group boxy-arm-facts (end-effector-link
                                robot-tool-frame
                                arm-joints arm-links
                                gripper-joint
                                gripper-link
                                standard-to-particular-gripper-transform
                                ;; robot-arms-parking-joint-states
                                ;; robot-arms-carrying-joint-states
                                robot-joint-states)

  (<- (end-effector-link boxy :left "left_arm_7_link"))
  (<- (end-effector-link boxy :right "right_arm_7_link"))

  (<- (robot-tool-frame boxy :left "left_gripper_tool_frame"))
  (<- (robot-tool-frame boxy :right "right_gripper_tool_frame"))

  (<- (arm-joints boxy :left ("left_arm_0_joint"
                              "left_arm_1_joint"
                              "left_arm_2_joint"
                              "left_arm_3_joint"
                              "left_arm_4_joint"
                              "left_arm_5_joint"
                              "left_arm_6_joint")))
  (<- (arm-joints boxy :right ("right_arm_0_joint"
                               "right_arm_1_joint"
                               "right_arm_2_joint"
                               "right_arm_3_joint"
                               "right_arm_4_joint"
                               "right_arm_5_joint"
                               "right_arm_6_joint")))

  (<- (arm-links boxy :left ("left_arm_1_link"
                             "left_arm_2_link"
                             "left_arm_3_link"
                             "left_arm_4_link"
                             "left_arm_5_link"
                             "left_arm_6_link"
                             "left_arm_7_link")))
  (<- (arm-links boxy :right ("right_arm_1_link"
                              "right_arm_2_link"
                              "right_arm_3_link"
                              "right_arm_4_link"
                              "right_arm_5_link"
                              "right_arm_6_link"
                              "right_arm_7_link")))

  (<- (gripper-joint boxy :left "left_gripper_joint"))
  (<- (gripper-joint boxy :right "right_gripper_joint"))

  (<- (gripper-link boxy :left ?link)
    (bound ?link)
    (lisp-fun search "left_gripper" ?link ?pos)
    (lisp-pred identity ?pos))
  (<- (gripper-link boxy :right ?link)
    (bound ?link)
    (lisp-fun search "right_gripper" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (standard-to-particular-gripper-transform boxy ?transform)
    (symbol-value *standard-to-boxy-gripper-transform* ?transform))

  (<- (robot-joint-states boxy :arm :left :carry ?joint-states)
    (symbol-value *left-parking-joint-states* ?joint-states))

  (<- (robot-joint-states boxy :arm :right :carry ?joint-states)
    (symbol-value *right-parking-joint-states* ?joint-states))

  (<- (robot-joint-states boxy :arm :left :park ?joint-states)
    (symbol-value *left-parking-joint-states* ?joint-states))

  (<- (robot-joint-states boxy :arm :right :park ?joint-states)
    (symbol-value *right-parking-joint-states* ?joint-states))

  ;; (<- (robot-arms-parking-joint-states boxy ?joint-states)
  ;;   (symbol-value *right-parking-joint-states* ?right-joint-states)
  ;;   (symbol-value *left-parking-joint-states* ?left-joint-states)
  ;;   (append ?right-joint-states ?left-joint-states ?joint-states))

  ;; (<- (robot-arms-parking-joint-states boxy ?joint-states :left)
  ;;   (symbol-value *left-parking-joint-states* ?joint-states))

  ;; (<- (robot-arms-parking-joint-states boxy ?joint-states :right)
  ;;   (symbol-value *right-parking-joint-states* ?joint-states))

  ;; (<- (robot-arms-carrying-joint-states boxy ?joint-states)
  ;;   (symbol-value *right-carrying-joint-states* ?right-joint-states)
  ;;   (symbol-value *left-carrying-joint-states* ?left-joint-states)
  ;;   (append ?right-joint-states ?left-joint-states ?joint-states))

  ;; (<- (robot-arms-carrying-joint-states boxy ?joint-states :left)
  ;;   (symbol-value *left-carrying-joint-states* ?joint-states))

  ;; (<- (robot-arms-carrying-joint-states boxy ?joint-states :right)
  ;;   (symbol-value *right-carrying-joint-states* ?joint-states))
  )
