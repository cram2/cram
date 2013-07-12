;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :pr2-itasc-pm)

;; the actual content and filling of the database
(defun fill-demo-database ()
  (fill-itasc-object-database)
  (fill-robot-database)
  (fill-itasc-tasks))

(defun fill-itasc-object-database ()
  (clear-itasc-object-list)
  (add-itasc-object
   :object-name "cupboard"
   :object-type "iTasc::FixedObject"
   :frames (list "cupboard_base" "upper_drawer"))
  (add-itasc-object
   :object-name "upper_drawer"
   :object-type "iTaSC::FixedObject"
   :frames (list "upper_drawer" "drawer_handle")))
  
(defun fill-robot-database ()
  (clear-itasc-robot-list)
  (add-itasc-robot
   :robot-name "Rubens"
   :robot-type "iTaSC::pr2Robot"
   :kinematic-chains
   (list (make-robot-kinematic-chain
          :chain-name "left arm"
          :robot-joint-weights
          (make-robot-joint-weight-standard-list
           :joint-names (list "l_shoulder_pan_joint" "l_shoulder_lift_joint" "l_upper_arm_roll_joint" "l_elbow_flex_joint" "l_forearm_roll_joint" "l_wrist_flex_joint" "l_wrist_roll_joint")))
         (make-robot-kinematic-chain
          :chain-name "right arm"
          :robot-joint-weights
          (make-robot-joint-weight-standard-list
           :joint-names (list "r_shoulder_pan_joint" "r_shoulder_lift_joint" "r_upper_arm_roll_joint" "r_elbow_flex_joint" "r_forearm_roll_joint" "r_wrist_flex_joint" "r_wrist_roll_joint")))
         (make-robot-kinematic-chain
          :chain-name "torso"
          :robot-joint-weights
          (make-robot-joint-weight-list
           :joint-names (list "torso_lift_joint")
           :weights (list 0.0))))
   :frames (list "base_link" "base_footprint" "l_gripper_tool_frame" "r_gripper_tool_frame"))
  (add-itasc-robot
   :robot-name "James"
   :robot-type "pr2Robot")
  :frames (list "base_link" "gripper"))

(defun fill-itasc-tasks ()
  (clear-itasc-tasks)
  (add-itasc-task
   (make-instance 'itasc-task
                  :name "robot base avoid cupboard"
                  :output-type :feature_constraints
                  :vkc (make-vkc
                        :chain-joints (append
                                       (make-cylindrical-coordinate-system)
                                       (make-rpy-representation))
                        :object1 (make-object-frame
                                  :object-name "cupboard"
                                  :frame-name "cupboard_base")
                        :object2 (make-object-frame
                                  :object-name "Rubens"
                                  :frame-name "base_link"))
                  :joint-constraints nil
                  :feature-constraints (list
                                        (make-constraint
                                         :constraint-name "stay above value"
                                         :referred-joint "radius"
                                         :operator :greater_operator
                                         :value 0.5))
                  :priority 1))
  (add-itasc-task
   (make-instance 'itasc-task
                  :name "right gripper into bunny pose"
                  :output-type :feature_constraints
                  :vkc (make-vkc
                        :chain-joints (append
                                       (make-cartesian-coordinate-system)
                                       (make-rpy-representation))
                        :object1 (make-object-frame
                                  :object-name "Rubens"
                                  :frame-name "base_link")
                        :object2 (make-object-frame
                                  :object-name "Rubens"
                                  :frame-name "r_gripper_tool_frame"))
                  :joint-constraints nil
                  :feature-constraints (list
                                        (make-constraint
                                         :constraint-name "go to the front"
                                         :referred-joint "x"
                                         :value 0.45)
                                        (make-constraint
                                         :constraint-name "go left"
                                         :referred-joint "y"
                                         :value -0.04)
                                        (make-constraint
                                         :constraint-name "go up"
                                         :referred-joint "z"
                                         :value 1.0)
                                        (make-constraint
                                         :constraint-name "roll"
                                         :referred-joint "roll"
                                         :value 0.0)
                                        (make-constraint
                                         :constraint-name "pitch"
                                         :referred-joint "pitch"
                                         :value 0.0)
                                        (make-constraint
                                         :constraint-name "yaw"
                                         :referred-joint "yaw"
                                         :value 0.0))
                  :priority 1))
  (add-itasc-task
   (make-instance 'itasc-task
                  :name "left gripper into bunny pose"
                  :output-type :feature_constraints
                  :vkc (make-vkc
                        :chain-joints (append
                                       (make-cartesian-coordinate-system)
                                       (make-rpy-representation))
                        :object1 (make-object-frame
                                  :object-name "Rubens"
                                  :frame-name "base_link")
                        :object2 (make-object-frame
                                  :object-name "Rubens"
                                  :frame-name "l_gripper_tool_frame"))
                  :joint-constraints nil
                  :feature-constraints (list
                                        (make-constraint
                                         :constraint-name "go to the front"
                                         :referred-joint "x"
                                         :value 0.45)
                                        (make-constraint
                                         :constraint-name "go left"
                                         :referred-joint "y"
                                         :value 0.04)
                                        (make-constraint
                                         :constraint-name "go up"
                                         :referred-joint "z"
                                         :value 1.0)
                                        (make-constraint
                                         :constraint-name "roll"
                                         :referred-joint "roll"
                                         :value 0.0)
                                        (make-constraint
                                         :constraint-name "pitch"
                                         :referred-joint "pitch"
                                         :value 0.0)
                                        (make-constraint
                                         :constraint-name "yaw"
                                         :referred-joint "yaw"
                                         :value 0.0))
                  :priority 1)))

(defun make-cylindrical-coordinate-system ()
  (make-chain-joint-list
   :joint-names (list "phi" "radius" "height")
   :joint-types (list :rotation_z :translation_x :translation_z)))

(defun make-cartesian-coordinate-system ()
  (make-chain-joint-list
   :joint-names (list "x" "y" "z")
   :joint-types (list :translation_x :translation_y :translation_z)))

(defun make-rpy-representation ()
  (make-chain-joint-list
   :joint-names (list "roll" "pitch" "yaw")
   :joint-types (list :rotation_x :rotation_y :rotation_z)))

;; using the database
(defun goto-bunny-config ()
  (let ((bunny-action (create-itasc-action
                       :tasks (assemble-tasks-msg-vector
                               (list
                                "right gripper into bunny pose"
                                "left gripper into bunny pose"))
                       :robot-joint-weights (make-robot-joint-weights-msg-vector
                                             (assemble-robot-joint-weights
                                              :robot-name "Rubens"
                                              :chain-names (list "torso"
                                                                 "left arm"
                                                                 "right arm")))
                       :objects (assemble-itasc-object-msg-vector 
                                 (list "cupboard" "upper_drawer")))))
    (perform-itasc-motion bunny-action)))