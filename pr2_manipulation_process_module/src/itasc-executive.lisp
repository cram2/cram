;;; Copyright (c) 2012, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :pr2-manip-pm)

;; the actual content and filling of the database
(defun fill-demo-database ()
  (fill-itasc-object-database)
  (fill-robot-database)
  (fill-itasc-tasks))

(defun fill-itasc-object-database ()
  (clear-itasc-object-list)
  (add-itasc-object
   :object-name "cupboard1"
   :object-type "cupboard"
   :frames (list "drawer-handle" "base_link"))
  (add-itasc-object
   :object-name "cupboard-top1"
   :object-type "cupboard-top"
   :frames (list "cupboard-top1"))
  (add-itasc-object
   :object-name "drawer1"
   :object-type "drawer"
   :frames (list "drawer1")))

(defun fill-robot-database ()
  (clear-itasc-robot-list)
  (add-itasc-robot
   :robot-name "Rubens"
   :robot-type "pr2Robot"
   :kinematic-chains
   (list (make-robot-kinematic-chain
          :chain-name "left arm"
          :robot-joint-weights
          (make-robot-joint-weight-standard-list
           :joint-names (list "joint0" "joint1" "joint2")))
         (make-robot-kinematic-chain
          :chain-name "right arm"
          :robot-joint-weights
          (make-robot-joint-weight-standard-list
           :joint-names (list "joint4" "joint5" "joint6")))
         (make-robot-kinematic-chain
          :chain-name "torso"
          :robot-joint-weights
          (make-robot-joint-weight-list
           :joint-names (list "torso-joint")
           :weights (list 0.0))))
   :frames (list "base_link" "left_gripper" "right_gripper"))
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
                                  :object-name "cupboard-top1"
                                  :frame-name "cupboard-top1")
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
                                  :frame-name "left_gripper"))
                  :joint-constraints nil
                  :feature-constraints (list
                                        (make-constraint
                                         :constraint-name "go to the front"
                                         :referred-joint "x"
                                         :value 0.5)
                                        (make-constraint
                                         :constraint-name "go left"
                                         :referred-joint "y"
                                         :value 0.2)
                                        (make-constraint
                                         :constraint-name "go up"
                                         :referred-joint "z"
                                         :value 1.2)
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
                                "robot base avoid cupboard"
                                "left gripper into bunny pose"))
                       :robot-joint-weights (make-robot-joint-weights-msg-vector
                                             (assemble-robot-joint-weights
                                              :robot-name "Rubens"
                                              :chain-names (list "torso"
                                                                 "left arm"
                                                                 "right arm")))
                       :objects (assemble-itasc-object-msg-vector 
                                 (list "cupboard-top1")))))
    (perform-itasc-motion bunny-action)))