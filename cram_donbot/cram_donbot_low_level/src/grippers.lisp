;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :donbot-ll)

(defparameter *gripper-action-timeout* 20.0 "in seconds")

(defparameter *gripper-velocity-convergence-delta* 0.00000001 "in meters/second")
(defparameter *gripper-convergence-delta* 0.005 "in meters")

(defun make-gripper-action-clients ()
  (actionlib-client:make-simple-action-client
   'gripper-home-action
   "wsg50/home_gripper_action"
   "slipping_control_msgs/HomeGripperAction"
   *gripper-action-timeout*)

  (actionlib-client:make-simple-action-client
   'gripper-grasp-action
   "wsg50/grasp_action"
   "slipping_control_msgs/GraspAction"
   *gripper-action-timeout*)

  (actionlib-client:make-simple-action-client
   'gripper-zero-left-action
   "wsg50/finger0/compute_bias_action"
   "sun_tactile_common/ComputeBiasAction"
   *gripper-action-timeout*)
  (actionlib-client:make-simple-action-client
   'gripper-zero-right-action
   "wsg50/finger1/compute_bias_action"
   "sun_tactile_common/ComputeBiasAction"
   *gripper-action-timeout*))

(roslisp-utilities:register-ros-init-function make-gripper-action-clients)

(defun call-gripper-set-object-parameters-service (object-parameter)
  (let ((service-name-0 "wsg50/ls_0/change_params")
        (service-name-1 "wsg50/ls_1/change_params"))
    (cpl:with-failure-handling
        ((roslisp::ros-rpc-error (e)
           (format t "ROS communication error occured!~%~a~%Retrying...~%~%" e)
           (loop until (roslisp:wait-for-service service-name-0 10)
                 do (roslisp:ros-info (donbot-ll change-params)
                                      "Waiting for ~a service server..."
                                      service-name-0))
           (loop until (roslisp:wait-for-service service-name-1 10)
                 do (roslisp:ros-info (donbot-ll change-params)
                                      "Waiting for ~a service server..."
                                      service-name-1))
           (cpl:retry)))
      (roslisp:call-service service-name-0 'slipping_control_msgs-srv:chlsparams
                            :mu object-parameter)
      (roslisp:call-service service-name-1 'slipping_control_msgs-srv:chlsparams
                            :mu object-parameter))))


(defun make-gripper-home-action-goal ()
  (roslisp:make-message
   'slipping_control_msgs-msg:HomeGripperGoal))

(defun call-gripper-home-action ()
  (actionlib-client:call-simple-action-client
   'gripper-home-action
   :action-goal (make-gripper-home-action-goal)))


(defun make-gripper-zero-action-goal ()
  (roslisp:make-message
   'sun_tactile_common-msg:ComputeBiasGoal))

(defun call-gripper-zero-actions ()
  (actionlib-client:call-simple-action-client
   'gripper-zero-left-action
   :action-goal (make-gripper-zero-action-goal))
  (actionlib-client:call-simple-action-client
   'gripper-zero-right-action
   :action-goal (make-gripper-zero-action-goal)))


(defun make-gripper-grasp-action-goal ()
  (roslisp:make-message
   'slipping_control_msgs-msg:GraspGoal
   :desired_force 5.0))

(defun call-gripper-grasp-action ()
  (actionlib-client:call-simple-action-client
   'gripper-grasp-action
   :action-goal (make-gripper-grasp-action-goal)))




(defun call-gripper-action (&key action-type-or-position left-or-right
                              effort-but-actually-slippage-parameter)
   (declare (type (or keyword list) left-or-right)
            (type (or null number) effort-but-actually-slippage-parameter)
            (type (or null number keyword) action-type-or-position))
   "Goal position is given in meters."

  (unless (eq left-or-right :left)
    (roslisp:ros-warn (donbot-ll gripper)
                      "Any gripper except left one is not supported.~%~
                       Defaulting to left gripper.")
    (setf left-or-right :left))

  (unless effort-but-actually-slippage-parameter
    (roslisp:ros-warn (donbot-ll gripper)
                      "effort-but-actually-slippage-parameter was not given..~%~
                       Defaulting to 0.72.")
    (setf effort-but-actually-slippage-parameter 0.72))

  (etypecase action-type-or-position
    (number
     (roslisp:ros-warn (donbot-ll gripper)
                       "Gripper goals with numeric positions are not supported.~%~
                        Simply opening the gripper.")
     (call-gripper-home-action))
    (keyword
     (ecase action-type-or-position
       (:open
        (call-gripper-home-action))
       (:close
        (roslisp:ros-warn (donbot-ll gripper)
                          "Gripper goals to close gripper are not supported.~%~
                           Not doing anything."))
       (:grip
        (call-gripper-zero-actions)
        (call-gripper-set-object-parameters-service
         effort-but-actually-slippage-parameter)
        (call-gripper-grasp-action))))))
