;;;
;;; Copyright (c) 2019,  Vanessa Hassouna <hassouna@uni-bremen.de>
;;;                      Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                    
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
   (cl-transforms:make-3d-vector 0.011999988555908198d0 0.0d0 0.21400000508626305d0)
   (cl-transforms:make-quaternion 0.0d0 0.0d0 -2.3205206744697995d-8 -0.9999999999999998d0)))

(defparameter *standard-to-hsrb-gripper-transform*
  (cl-transforms-stamped:make-identity-transform))

(def-fact-group hsrb-arm-facts (end-effector-link
                                robot-tool-frame
                                arm-joints arm-links
                                gripper-joint
                                gripper-link
                                standard-to-particular-gripper-transform
                                tcp-in-ee-pose)

  (<- (end-effector-link hsrb :left "wrist_roll_link"))
  (<- (end-effector-link hsrb :right "ATTENTION please don't use this frame"))


  (<- (robot-tool-frame hsrb :left "gripper_tool_joint"))
  (<- (robot-tool-frame hsrb :right "ATTENTION please don't use this frame"))

  (<- (arm-joints hsrb :left ("arm_flex_joint"
                              "arm_roll_joint"
                              "wrist_flex_joint"
                              "wrist_roll_joint")))

  (<- (arm-links hsrb :left ("arm_flex_link"
                             "arm_roll_link"
                             "wrist_flex_link"
                             "wrist_roll_link")))

  (<- (arm-joints hsrb :right ("ATTENTION please don't use this frame")))

  (<- (arm-links hsrb :right ("ATTENTION please don't use this frame")))

  (<- (gripper-joint hsrb :left "hand_motor_joint"))

  (<- (gripper-joint hsrb :right "ATTENTION please don't use this frame"))

  (<- (gripper-link hsrb :left ?link)
    (bound ?link)
    (lisp-fun search "hand_palm_link" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (gripper-link hsrb :right ?link)
    (bound ?link)
    (lisp-fun search "ATTENTION please don't use this frame" ?link ?pos)
    (lisp-pred identity ?pos))


  (<- (standard-to-particular-gripper-transform hsrb ?transform)
    (symbol-value *standard-to-hsrb-gripper-transform* ?transform))

  (<- (tcp-in-ee-pose hsrb ?transform)
    (symbol-value *tcp-in-ee-pose* ?transform)))
