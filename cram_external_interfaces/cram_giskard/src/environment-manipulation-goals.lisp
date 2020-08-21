;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :giskard)

(defun make-environment-manipulation-goal (open-or-close arm
                                           handle-link joint-state
                                           prefer-base)
  (declare (type keyword open-or-close arm)
           (type symbol handle-link)
           (type (or number null) joint-state)
           (type boolean prefer-base))
  (make-giskard-goal
   :constraints (list
                 (when prefer-base (make-prefer-base-constraint))
                 (make-open-or-close-constraint
                  open-or-close arm handle-link joint-state))
   :collisions (make-constraints-vector
                (make-avoid-all-collision 0.05)
                (ecase open-or-close
                  (:open (make-allow-hand-collision
                          arm (rob-int:get-environment-name) handle-link))
                  (:close (make-allow-arm-collision
                           arm (rob-int:get-environment-name)))))))

(defun call-environment-manipulation-action (&key open-or-close arm
                                               handle-link joint-state
                                               prefer-base
                                               action-timeout)
  (declare (type keyword open-or-close arm)
           (type symbol handle-link)
           (type (or number null) joint-state action-timeout)
           (type boolean prefer-base))
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'giskard-action
       :action-goal (make-environment-manipulation-goal
                     open-or-close arm handle-link joint-state prefer-base)
       :action-timeout action-timeout)
    (ensure-goal-reached status)
    (values result status)))


#+the-plan
(
 (setf (btr:joint-state (btr:get-environment-object) "iai_fridge_door_joint")
       0.0)
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type parking-arms))))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type releasing)
              (gripper right))))
 (giskard::call-grasp-bar-action :arm :right :bar-length 0.8)
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type gripping)
              (gripper right))))
 (giskard::call-environment-manipulation-action :open-or-close :open
                                                :arm :right
                                                :handle-link :iai-fridge-door-handle)
 (giskard:call-giskard-cartesian-action
  :goal-pose-right (cl-transforms-stamped:make-pose-stamped "r_gripper_tool_frame" 0.0 (cl-transforms:make-3d-vector -0.1 0 0) (cl-transforms:make-identity-rotation))
  :collision-mode :allow-all)
 (coe:on-event (make-instance 'cpoe:robot-state-changed))
 (setf (btr:joint-state (btr:get-environment-object)
                        "iai_fridge_door_joint") 1.45)
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 (giskard::call-environment-manipulation-action :open-or-close :close
                                                :arm :right
                                                :handle-link :iai-fridge-door-handle)
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type releasing)
              (gripper right))))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type parking-arms))))
 (setf (btr:joint-state (btr:get-environment-object) "iai_fridge_door_joint")
       0.0)
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 )

#+the-dishwasher-plan
(
 (setf (btr:joint-state (btr:get-environment-object)
                        "sink_area_dish_washer_door_joint")
       0.0)
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type parking-arms))))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type releasing)
              (gripper right))))
 (giskard::call-grasp-bar-action
  :arm :right
  :bar-axis (cl-transforms-stamped:make-vector-stamped
             "iai_kitchen/sink_area_dish_washer_door_handle" 0.0
             (cl-transforms:make-3d-vector 0 1 0))
  :bar-perpendicular-axis
  (cl-transforms-stamped:make-vector-stamped
   "iai_kitchen/sink_area_dish_washer_door_handle" 0.0
   (cl-transforms:make-3d-vector 0 0 1))
  :bar-center (cl-transforms-stamped:make-point-stamped
               "iai_kitchen/sink_area_dish_washer_door_handle" 0.0
               (cl-transforms:make-3d-vector 0.0 0 0))
  :bar-length 0.2)
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type gripping)
              (gripper right))))
 (giskard::call-environment-manipulation-action
  :open-or-close :open
  :arm :right
  :handle-link :sink-area-dish-washer-door-handle
  :joint-state (cram-math:degrees->radians 35))
 (coe:on-event (make-instance 'cpoe:robot-state-changed))
 (setf (btr:joint-state (btr:get-environment-object)
                        "sink_area_dish_washer_door_joint")
       (cram-math:degrees->radians 35))
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 (giskard::call-environment-manipulation-action
  :open-or-close :close
  :arm :right
  :handle-link :sink-area-dish-washer-door-handle)
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type releasing)
              (gripper right))))
 (pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type parking-arms))))
 (setf (btr:joint-state (btr:get-environment-object)
                        "sink_area_dish_washer_door_joint")
       (cram-math:degrees->radians 0))
 (btr-belief::publish-environment-joint-state
  (btr:joint-states (btr:get-environment-object)))
 )
