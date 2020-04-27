;;;
;;; Copyright (c) 2019,  Vanessa Hassouna <hassouna@uni-bremen.de>
;;;                      Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-hsrb-description)

(defparameter *tcp-in-ee-pose*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.23090001999999998d0)
   (cl-tf:make-quaternion 0.0d0 0.0d0 0.7071067811865475d0 0.7071067811865476d0)))

(defparameter *standard-to-hsrb-gripper-transform*
  (cl-transforms-stamped:make-identity-transform))

(defparameter *left-parking-joint-states*
  '(("arm_flex_joint" 0)
    ("arm_roll_joint" 1.5)
    ("wrist_flex_joint" -1.85)
    ("wrist_roll_joint" 0)))

(defun get-hand-link-names (arm)
  (ecase arm
    (:left (list "hand_l_distal_link"
                 "hand_l_spring_proximal_link"
                 "hand_palm_link"
                 "hand_r_distal_link"
                 "hand_r_spring_proximal_link"))))


(def-fact-group hsrb-arm-facts (arm
                                end-effector-link
                                robot-tool-frame
                                arm-joints arm-links
                                gripper-joint gripper-link
                                gripper-meter-to-joint-multiplier
                                standard-to-particular-gripper-transform
                                robot-joint-states
                                tcp-in-ee-pose)

  (<- (arm hsrb :left))

  (<- (end-effector-link hsrb :left "wrist_roll_link"))
  ;; cram-tf needs the right link/joint, for now we can do it like this
  (<- (end-effector-link hsrb :right "ERROR: HSRB does not have a right arm!"))

  (<- (robot-tool-frame hsrb :left "gripper_tool_joint"))
  (<- (robot-tool-frame hsrb :right "ERROR: HSRB does not have a right arm!"))

  (<- (arm-joints hsrb :left ("arm_flex_joint"
                              "arm_roll_joint"
                              "wrist_flex_joint"
                              "wrist_roll_joint")))
  (<- (arm-joints hsrb :right ("ERROR: HSRB does not have a right arm!")))

  (<- (arm-links hsrb :left ("arm_flex_link"
                             "arm_roll_link"
                             "wrist_flex_link"
                             "wrist_roll_link")))
  (<- (arm-links hsrb :right ("ERROR: HSRB does not have a right arm!")))

  (<- (gripper-joint hsrb :left "hand_motor_joint"))
  (<- (gripper-joint hsrb :right "ERROR: HSRB does not have a right arm!"))

  (<- (gripper-link hsrb :left ?link)
    (bound ?link)
    (lisp-fun search "hand" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (gripper-meter-to-joint-multiplier hsrb 1.0))

  (<- (standard-to-particular-gripper-transform hsrb ?transform)
    (symbol-value *standard-to-hsrb-gripper-transform* ?transform))

  (<- (robot-joint-states hsrb :arm :left :park ?joint-states)
    (symbol-value *left-parking-joint-states* ?joint-states))

  (<- (robot-joint-states hsrb :arm :left :carry ?joint-states)
    (symbol-value *left-parking-joint-states* ?joint-states))

  (<- (tcp-in-ee-pose hsrb ?pose)
    (symbol-value *tcp-in-ee-pose* ?pose)))
