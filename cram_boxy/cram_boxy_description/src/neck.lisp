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

(defparameter *neck-good-looking-down-state*
  '(("neck_shoulder_pan_joint" -1.176d0)
    ("neck_shoulder_lift_joint" -3.1252d0)
    ("neck_elbow_joint" -0.8397d0)
    ("neck_wrist_1_joint" 0.83967d0)
    ("neck_wrist_2_joint" 1.1347d0)
    ("neck_wrist_3_joint" -0.0266d0)))

(defparameter *neck-good-looking-left-state*
  '(("neck_shoulder_pan_joint" -0.776d0)
    ("neck_shoulder_lift_joint" -3.1252d0)
    ("neck_elbow_joint" -0.8397d0)
    ("neck_wrist_1_joint" 0.83967d0)
    ("neck_wrist_2_joint" 1.1347d0)
    ("neck_wrist_3_joint" -0.0266d0)))

(defparameter *neck-parking-joint-states*
  '(("neck_shoulder_pan_joint" -1.3155d0)
    ("neck_shoulder_lift_joint" -1.181355d0)
    ("neck_elbow_joint" -1.9562d0)
    ("neck_wrist_1_joint" 0.142417d0)
    ("neck_wrist_2_joint" 1.13492d0)
    ("neck_wrist_3_joint" 0.143467d0)))

(def-fact-group boxy-neck-facts (robot-pan-tilt-links
                                 robot-pan-tilt-joints
                                 ;; robot-neck-parking-joint-states
                                 ;; robot-neck-looking-joint-states
                                 robot-joint-states)

  (<- (robot-pan-tilt-links boxy
                            "neck_shoulder_link"
                            "neck_upper_arm_link"
                            "neck_forearm_link"
                            "neck_wrist_1_link"
                            "neck_wrist_2_link"
                            "neck_wrist_3_link"))

  (<- (robot-pan-tilt-joints boxy
                             "neck_shoulder_pan_joint"
                             "neck_shoulder_lift_joint"
                             "neck_elbow_joint"
                             "neck_wrist_1_joint"
                             "neck_wrist_2_joint"
                             "neck_wrist_3_joint"))

  (<- (robot-joint-states boxy :neck ?there-is-only-one-neck :park ?joint-states)
    (symbol-value *neck-parking-joint-states* ?joint-states))

  (<- (robot-joint-states boxy :neck ?there-is-only-one-neck :down ?joint-states)
    (symbol-value *neck-good-looking-down-state* ?joint-states))

  (<- (robot-joint-states boxy :neck ?there-is-only-one-neck :down-left ?joint-states)
    (symbol-value *neck-good-looking-left-state* ?joint-states))

  ;; (<- (robot-neck-parking-joint-states boxy ?joint-states)
  ;;   (symbol-value *neck-parking-joint-states* ?joint-states))

  ;; (<- (robot-neck-looking-joint-states boxy ?joint-states)
  ;;   (symbol-value *neck-good-looking-left-state* ?joint-states))
  )
