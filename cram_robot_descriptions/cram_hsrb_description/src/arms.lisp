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
   (cl-transforms:make-3d-vector 0 0 0.02)
   (cl-transforms-stamped:make-quaternion 0 0 0 1)))

(defparameter *standard-to-hsrb-gripper-transform*
  (cl-transforms:make-transform
   (cl-transforms:make-identity-vector)
   (cl-transforms:make-quaternion 0 0 1 1)))



(def-fact-group hsrb-arm-facts (arm
                                arm-joints arm-links
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

  (<- (arm :hsrb :left))

  (<- (arm-joints :hsrb :left ("arm_flex_joint"
                               "arm_roll_joint"
                               "wrist_flex_joint"
                               "wrist_roll_joint")))

  (<- (arm-links :hsrb :left ("arm_flex_link"
                              "arm_roll_link"
                              "wrist_flex_link"
                              "wrist_roll_link"
                              "hand_palm_link")))

  (<- (hand-links :hsrb :left ("hand_l_proximal_link"
                               "hand_r_proximal_link"
                               "hand_l_distal_link"
                               "hand_r_distal_link"
                               "hand_l_finger_tip_frame"
                               "hand_r_finger_tip_frame")))

  (<- (hand-link :hsrb :left ?link)
    (bound ?link)
    (lisp-fun search "hand" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (hand-finger-link :hsrb ?arm ?link)
    (bound ?link)
    (hand-link :hsrb ?arm ?link)
    (lisp-fun search "finger" ?link ?pos)
    (not (lisp-pred identity ?pos)))

  (<- (gripper-joint :hsrb :left "hand_l_proximal_joint"))
  (<- (gripper-joint :hsrb :left "hand_r_proximal_joint"))

  (<- (gripper-meter-to-joint-multiplier :hsrb 5))
  (<- (gripper-minimal-position :hsrb ?_ 0.8))


  (<- (standard<-particular-gripper-transform :hsrb ?transform)
    (symbol-value *standard-to-hsrb-gripper-transform* ?transform))

  (<- (end-effector-link :hsrb :left "hand_palm_link"))

  (<- (robot-tool-frame :hsrb :left "hand_gripper_tool_frame"))
  (<- (robot-tool-frame :hsrb :right "hand_gripper_tool_frame"))
  

  (<- (tcp-in-ee-pose :hsrb ?pose)
    (symbol-value *tcp-in-ee-pose* ?pose))

  (<- (robot-joint-states :hsrb :arm :left :carry 
                          (("arm_flex_joint" -1.7957002015087948)
                           ("arm_roll_joint" -0.009193682596488978)
                           ("wrist_flex_joint" 0.22629382758447783)
                           ("wrist_roll_joint" -1.561914318814292))))
  (<- (robot-joint-states :hsrb :arm :left :park
                          (("arm_flex_joint" 0)
                           ("arm_roll_joint" 1.5)
                           ("wrist_flex_joint" -1.9)
                           ("wrist_roll_joint" 0))))
  
  (<- (robot-joint-states :hsrb :arm :left :park ?joint-states)
    (robot-joint-states :hsrb :arm :right :carry ?joint-states)))



  ;; hand proximal joint

  ;; hand proximal  joint
  ;; (<- (gripper-minimal-position :hsrb "hand_l_proximal_joint" 0.65))
  ;; (<- (gripper-maximal-position :hsrb "hand_r_proximal_joint" 0.75))
  ;; (<- (gripper-convergence-delta :hsrb "hand_l_spring_proximal_joint" 0.001))

  ;; ;; hand distal joint
  ;; (<- (gripper-minimal-position :hsrb "hand_l_distal_joint" -1.3))
  ;; (<- (gripper-maximal-position :hsrb "hand_l_distal_joint" 0.70))
  ;; (<- (gripper-convergence-delta :hsrb "hand_l_distal_joint" 0.001))

  ;; ;; hand mimic distal joint
  ;; (<- (gripper-minimal-position :hsrb "hand_l_mimic_distal_joint" -0.74))
  ;; (<- (gripper-maximal-position :hsrb "hand_l_mimic_distal_joint" 0.03))
  ;; (<- (gripper-convergence-delta :hsrb "hand_l_mimic_distal_joint" 0.001))

  ;; ;; wrist flex joint
  ;; ;; (<- (gripper-minimal-position :hsrb "wrist_flex_joint" -1.17119))
  ;; ;; (<- (gripper-maximal-position :hsrb "wrist_flex_joint" -1.1689111))
  ;; (<- (gripper-minimal-position :hsrb "wrist_flex_joint" -1.75168))
  ;; (<- (gripper-maximal-position :hsrb "wrist_flex_joint" -1.5695))
  ;; (<- (gripper-convergence-delta :hsrb "wrist_flex_joint" 0.001))

  ;; ;; arm flex joint
  ;; (<- (gripper-minimal-position :hsrb "arm_flex_joint" -0.02986912640)) ;;13678903))
  ;; (<- (gripper-maximal-position :hsrb "arm_flex_joint" -1.168703))
  ;; (<- (gripper-convergence-delta :hsrb "arm_flex_joint" 0.00))

  ;; ;; default  
  ;; (<- (gripper-minimal-position :hsrb ?_ 0))
  ;; (<- (gripper-maximal-position :hsrb ?_ 1.24))
  ;; (<- (gripper-convergence-delta :hsrb ?_ 0.001))
