;;; Copyright (c) 2012, CRAM team
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

(in-package :cram-pr2-description)

(defparameter *tcp-in-ee-pose*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector 0.18 0 0)
   (cl-transforms-stamped:make-identity-rotation)))

(defparameter *standard-to-pr2-gripper-transform*
  (cl-transforms:make-transform
   (cl-transforms:make-identity-vector)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (0 0 1)
        (1 0 0)))))

;; (defparameter *right-parking-end-effector-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "torso_lift_link" 0.0
;;    (cl-transforms:make-3d-vector 0.3 -0.3 -0.23)
;;    (cl-transforms:euler->quaternion :ay (/ pi 2))))

;; (defparameter *right-parking-tcp-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "base_footprint"
;;    0.0
;;    (cl-transforms:make-3d-vector 0.4 -0.3 1.55)
;;    (cl-transforms:make-quaternion 0.029319081708036543d0 -0.018714920400581137d0
;;                                   0.5257710356470319d0 0.8499146788218482d0)))

;; (defparameter *left-parking-end-effector-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "torso_lift_link" 0.0
;;    (cl-transforms:make-3d-vector 0.3 0.3 -0.23)
;;    (cl-transforms:euler->quaternion :ay (/ pi 2))))

;; (defparameter *left-parking-tcp-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "base_footprint"
;;    0.0
;;    (cl-transforms:make-3d-vector 0.4 0.3 1.55)
;;    (cl-transforms:make-quaternion 0.9215513103717499d0 -0.387996037470125d0
;;                                   -0.014188589447636247d0 -9.701489976338351d-4)))





(defun get-arm-joint-names (arm)
  ;; TODO: the proper way to do this is to read them out of the srdl,
  ;; so that we don't need to write the same thing, consistently, in several places
  (ecase arm
    (:right (list "r_shoulder_pan_joint"
                  "r_shoulder_lift_joint"
                  "r_upper_arm_roll_joint"
                  "r_elbow_flex_joint"
                  "r_forearm_roll_joint"
                  "r_wrist_flex_joint"
                  "r_wrist_roll_joint"))
    (:left (list "l_shoulder_pan_joint"
                 "l_shoulder_lift_joint"
                 "l_upper_arm_roll_joint"
                 "l_elbow_flex_joint"
                 "l_forearm_roll_joint"
                 "l_wrist_flex_joint"
                 "l_wrist_roll_joint"))))

(defun get-arm-link-names (arm)
  ;; TODO: the proper way to do this is to read them out of the srdf,
  ;; so that we don't need to write the same thing, consistently, in several places
  (ecase arm
    (:left (list "l_shoulder_pan_link"
                 "l_shoulder_lift_link"
                 "l_upper_arm_roll_link"
                 "l_upper_arm_link"
                 "l_elbow_flex_link"
                 "l_forearm_roll_link"
                 "l_forearm_link"
                 "l_wrist_flex_link"
                 "l_wrist_roll_link"
                 "l_gripper_led_frame"
                 "l_gripper_motor_accelerometer_link"
                 "l_gripper_tool_frame"
                 "l_gripper_r_finger_link"
                 "l_gripper_r_finger_tip_link"
                 "l_gripper_l_finger_tip_frame"
                 "l_gripper_l_finger_link"
                 "l_gripper_l_finger_tip_link"
                 "l_gripper_motor_slider_link"
                 "l_gripper_motor_screw_link"
                 "l_gripper_palm_link"
                 "l_force_torque_link"
                 "l_force_torque_adapter_link"))
    (:right (list "r_gripper_palm_link"
                  "r_shoulder_pan_link"
                  "r_shoulder_lift_link"
                  "r_upper_arm_roll_link"
                  "r_upper_arm_link"
                  "r_elbow_flex_link"
                  "r_forearm_roll_link"
                  "r_forearm_link"
                  "r_wrist_flex_link"
                  "r_wrist_roll_link"
                  "r_gripper_led_frame"
                  "r_gripper_motor_accelerometer_link"
                  "r_gripper_tool_frame"
                  "r_gripper_r_finger_link"
                  "r_gripper_r_finger_tip_link"
                  "r_gripper_l_finger_tip_frame"
                  "r_gripper_l_finger_link"
                  "r_gripper_l_finger_tip_link"
                  "r_gripper_motor_slider_link"
                  "r_gripper_motor_screw_link"))))

(defun get-hand-link-names (arm)
  (ecase arm
    (:left (list "l_gripper_l_finger_tip_link"
                 "l_gripper_r_finger_tip_link"
                 "l_gripper_l_finger_link"
                 "l_gripper_r_finger_link"
                 "l_gripper_l_finger_tip_frame"
                 "l_gripper_palm_link"))
    (:right (list "r_gripper_l_finger_tip_link"
                  "r_gripper_r_finger_tip_link"
                  "r_gripper_l_finger_link"
                  "r_gripper_r_finger_link"
                  "r_gripper_l_finger_tip_frame"
                  "r_gripper_palm_link"))))

(def-fact-group pr2-arm-kinematics-facts (arm
                                          arm-links arm-joints
                                          hand-links hand-link hand-finger-link
                                          gripper-joint
                                          gripper-meter-to-joint-multiplier
                                          gripper-minimal-position
                                          gripper-convergence-delta
                                          standard<-particular-gripper-transform
                                          end-effector-link robot-tool-frame
                                          tcp-in-ee-pose
                                          robot-joint-states)

  (<- (arm :pr2 :right))
  (<- (arm :pr2 :left))

  (<- (arm-links :pr2 ?arm ?links)
    (lisp-fun get-arm-link-names ?arm ?links))

  (<- (arm-joints :pr2 ?arm ?joints)
    (lisp-fun get-arm-joint-names ?arm ?joints))

  (<- (hand-links :pr2 ?arm ?links)
    (lisp-fun get-hand-link-names ?arm ?links))

  (<- (hand-link :pr2 :left ?link)
    (bound ?link)
    (lisp-fun search "l_gripper" ?link ?pos)
    (lisp-pred identity ?pos))
  (<- (hand-link :pr2 :right ?link)
    (bound ?link)
    (lisp-fun search "r_gripper" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (hand-finger-link :pr2 ?arm ?link)
    (bound ?link)
    (hand-link :pr2 ?arm ?link)
    (lisp-fun search "finger" ?link ?pos)
    (lisp-pred identity ?pos))

  ;; (<- (gripper-joint :pr2 :left "l_gripper_joint"))
  ;; (<- (gripper-joint :pr2 :right "r_gripper_joint"))
  (<- (gripper-joint :pr2 :left "l_gripper_l_finger_joint"))
  (<- (gripper-joint :pr2 :left "l_gripper_r_finger_joint"))
  (<- (gripper-joint :pr2 :right "r_gripper_l_finger_joint"))
  (<- (gripper-joint :pr2 :right "r_gripper_r_finger_joint"))

  (<- (gripper-meter-to-joint-multiplier :pr2 5.0))
  (<- (gripper-minimal-position :pr2 ?_ 0.013))
  (<- (gripper-convergence-delta :pr2 ?_ 0.005))

  (<- (standard<-particular-gripper-transform :pr2 ?transform)
    (symbol-value *standard-to-pr2-gripper-transform* ?transform))

  (<- (end-effector-link :pr2 :left "l_wrist_roll_link"))
  (<- (end-effector-link :pr2 :right "r_wrist_roll_link"))

  (<- (robot-tool-frame :pr2 :left "l_gripper_tool_frame"))
  (<- (robot-tool-frame :pr2 :right "r_gripper_tool_frame"))

  (<- (tcp-in-ee-pose :pr2 ?transform)
    (symbol-value *tcp-in-ee-pose* ?transform))

  ;; (<- (end-effector-parking-pose :pr2 ?pose :left)
  ;;   (symbol-value *left-parking-end-effector-pose* ?pose))
  ;; (<- (end-effector-parking-pose :pr2 ?pose :right)
  ;;   (symbol-value *right-parking-end-effector-pose* ?pose))

  (<- (robot-joint-states :pr2 :arm :left :carry
                          (("l_shoulder_pan_joint" 1.9652919379395388d0)
                           ("l_shoulder_lift_joint" -0.26499816732737785d0)
                           ("l_upper_arm_roll_joint" 1.3837617139225473d0)
                           ("l_elbow_flex_joint" -2.1224566064321584d0)
                           ("l_forearm_roll_joint" 16.99646118944817d0)
                           ("l_wrist_flex_joint" -0.07350789589924167d0)
                           ("l_wrist_roll_joint" 0.0))))
  (<- (robot-joint-states :pr2 :arm :left :park ?joint-states)
    (robot-joint-states :pr2 :arm :left :carry ?joint-states))
  (<- (robot-joint-states :pr2 :arm :left :carry-top
                          ;; arm low -- collision avoidance hard atm
                          ;; (("l_shoulder_pan_joint" 1.3810115229719555d0)
                          ;;  ("l_shoulder_lift_joint" 1.1282870348994702d0)
                          ;;  ("l_upper_arm_roll_joint" 1.71d0)
                          ;;  ("l_elbow_flex_joint" -2.105735087282934d0)
                          ;;  ("l_forearm_roll_joint" 2.6581354736032257d0)
                          ;;  ("l_wrist_flex_joint" -1.9927790883777252d0)
                          ;;  ("l_wrist_roll_joint" 2.586184460547585d0))
                          (("l_shoulder_pan_joint" 1.0252138037286773d0)
                           ("l_shoulder_lift_joint" -0.06966848987919201d0)
                           ("l_upper_arm_roll_joint" 1.1765832782526544d0)
                           ("l_elbow_flex_joint" -1.9323726623855864d0)
                           ("l_forearm_roll_joint" 1.3824994377973336d0)
                           ("l_wrist_flex_joint" -1.8416233909065576d0)
                           ("l_wrist_roll_joint" 2.907373693068033d0))))
  (<- (robot-joint-states :pr2 :arm :left :carry-top-basket
                          (("l_shoulder_pan_joint" 1.0)
                           ("l_shoulder_lift_joint" -0.5)
                           ("l_upper_arm_roll_joint" 3.14)
                           ("l_elbow_flex_joint" -1.5)
                           ("l_forearm_roll_joint" -0.2)
                           ("l_wrist_flex_joint" -0.55))))
  (<- (robot-joint-states :pr2 :arm :left :hand-over
                          (("l_shoulder_pan_joint" -0.3)
                           ("l_shoulder_lift_joint" -0.5)
                           ("l_upper_arm_roll_joint" 3.14)
                           ("l_elbow_flex_joint" -1.3)
                           ("l_forearm_roll_joint" 0)
                           ("l_wrist_flex_joint" -0.75))))
  (<- (robot-joint-states :pr2 :arm :left :carry-side-gripper-vertical
                          (("l_shoulder_pan_joint" 1.2469064067488675d0)
                           ("l_shoulder_lift_joint" 0.013567714247075813d0)
                           ("l_upper_arm_roll_joint" 1.3837617139225473d0)
                           ("l_elbow_flex_joint" -2.105735087282934d0)
                           ("l_forearm_roll_joint" 0.8809236347313467d0)
                           ("l_wrist_flex_joint" -0.9276887874976607d0)
                           ("l_wrist_roll_joint" 2.3644392879261957d0))))
  (<- (robot-joint-states :pr2 :arm :left :carry-tray
                          (("l_shoulder_pan_joint" 0.5093452794519218d0)
                           ("l_shoulder_lift_joint" -0.3409151070391534d0)
                           ("l_upper_arm_roll_joint" 0.7237617139225473d0)
                           ("l_elbow_flex_joint" -1.1711686511880242d0)
                           ("l_forearm_roll_joint" 0.9029896286150922d0)
                           ("l_wrist_flex_joint" -1.5572577906669434d0)
                           ("l_wrist_roll_joint" 1.9999842405583337d0))))
  (<- (robot-joint-states :pr2 :arm :left :tucked
                          (("l_shoulder_pan_joint" 0.1709440184822959d0)
                           ("l_shoulder_lift_joint" 1.1472294789783886d0)
                           ("l_upper_arm_roll_joint" 1.9124515764640622d0)
                           ("l_elbow_flex_joint" -1.66700794841958d0)
                           ("l_forearm_roll_joint" 6.255471931555043d0)
                           ("l_wrist_flex_joint" -0.07476630774212056d0)
                           ("l_wrist_roll_joint" -14.7079336142174d0))))

  (<- (robot-joint-states :pr2 :arm :right :carry
                          (("r_shoulder_pan_joint" -1.712587449591307d0)
                           ("r_shoulder_lift_joint" -0.2567290370386635d0)
                           ("r_upper_arm_roll_joint" -1.4633501125737374d0)
                           ("r_elbow_flex_joint" -2.1221670650093913d0)
                           ("r_forearm_roll_joint" 1.7663253481913623d0)
                           ("r_wrist_flex_joint" -0.07942669250968948d0)
                           ("r_wrist_roll_joint" 0.05106258161229582d0))))
  (<- (robot-joint-states :pr2 :arm :right :park ?joint-states)
    (robot-joint-states :pr2 :arm :right :carry ?joint-states))
  (<- (robot-joint-states :pr2 :arm :right :carry-top
                          ;; arm low -- collision avoidance hard atm
                          ;; (("r_shoulder_pan_joint" -1.3810115229719555d0)
                          ;;  ("r_shoulder_lift_joint" 1.1282870348994702d0)
                          ;;  ("r_upper_arm_roll_joint" -1.7100000000000002d0)
                          ;;  ("r_elbow_flex_joint" -2.105735087282934d0)
                          ;;  ("r_forearm_roll_joint" -2.658135473603226d0)
                          ;;  ("r_wrist_flex_joint" -1.9927790883777252d0)
                          ;;  ("r_wrist_roll_joint" -2.5861844605475843d0))
                          (("r_shoulder_pan_joint" -1.183809044088452d0)
                           ("r_shoulder_lift_joint" -0.30680923151650064d0)
                           ("r_upper_arm_roll_joint" -1.6439096470662296d0)
                           ("r_elbow_flex_joint" -2.0632453854762955d0)
                           ("r_forearm_roll_joint" -1.3016362275774522d0)
                           ("r_wrist_flex_joint" -1.284945123721191d0)
                           ("r_wrist_roll_joint" -1.974782974061931d0))))
  (<- (robot-joint-states :pr2 :arm :right :carry-side-gripper-vertical
                          (("r_shoulder_pan_joint" -0.7524126363048715d0)
                           ("r_shoulder_lift_joint" -0.2d0)
                           ("r_upper_arm_roll_joint" -1.7100000000000002d0)
                           ("r_elbow_flex_joint" -2.1221670650093913d0)
                           ("r_forearm_roll_joint" -0.5d0)
                           ("r_wrist_flex_joint" -0.7163017685583473d0)
                           ("r_wrist_roll_joint" 0.37943005734078916d0))))
  (<- (robot-joint-states :pr2 :arm :right :carry-tray
                          (("r_shoulder_pan_joint" -0.5091745530984966d0)
                           ("r_shoulder_lift_joint" -0.3407352620327657d0)
                           ("r_upper_arm_roll_joint" -0.7233501125737374d0)
                           ("r_elbow_flex_joint" -1.1711969086974978d0)
                           ("r_forearm_roll_joint" -0.9033955737280515d0)
                           ("r_wrist_flex_joint" -1.5575236188499488d0)
                           ("r_wrist_roll_joint" 1.1418180038095498d0))))
  (<- (robot-joint-states :pr2 :arm :right :tucked
                          (("r_shoulder_pan_joint" -0.08181428617939712d0)
                           ("r_shoulder_lift_joint" 0.9781030555170612d0)
                           ("r_upper_arm_roll_joint" -1.4665572091011352d0)
                           ("r_elbow_flex_joint" -1.6859729116108224d0)
                           ("r_forearm_roll_joint" -27.72481374424779d0)
                           ("r_wrist_flex_joint" -0.10621948550701799d0)
                           ("r_wrist_roll_joint" 7.662671673625887d0)))))

