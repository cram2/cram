;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-ll)

(defvar *joint-state-sub* nil
  "Subscriber for robot's joint state topic.")

(defvar *robot-joint-states-msg* (cpl:make-fluent :name :robot-joint-states)
  "ROS message containing robot's current joint states.")

(defun init-joint-state-sub ()
  "Initializes *joint-state-sub*"
  (flet ((joint-state-sub-cb (joint-state-msg)
           (setf (cpl:value *robot-joint-states-msg*) joint-state-msg)))
    (setf *joint-state-sub*
          (roslisp:subscribe "joint_states"
                             "sensor_msgs/JointState"
                             #'joint-state-sub-cb))))

(defun destroy-joint-state-sub ()
  (setf *joint-state-sub* nil))

(roslisp-utilities:register-ros-init-function init-joint-state-sub)
(roslisp-utilities:register-ros-cleanup-function destroy-joint-state-sub)

(defclass joint-state ()
  ((name :reader joint-state-name
         :initarg :name
         :type string)
   (position :reader joint-state-position
             :initarg :position
             :type float)
   (velocity :reader joint-state-velocity
             :initarg :velocity
             :type float)
   (effort :reader joint-state-effort
           :initarg :effort
           :type float)))

(defun joint-states (names)
  "Returns the joint states of type JOINT-STATE + the corresponding timestamp
as multiple values."
  (let ((last-joint-state-msg (cpl:value *robot-joint-states-msg*)))
    (values
     (mapcar (lambda (name)
               (let ((index (position
                             name
                             (roslisp:msg-slot-value last-joint-state-msg :name)
                             :test #'string-equal)))
                 (when index
                   (make-instance 'joint-state
                     :name name
                     :position (aref (roslisp:msg-slot-value last-joint-state-msg :position)
                                     index)
                     :velocity (aref (roslisp:msg-slot-value last-joint-state-msg :velocity)
                                     index)
                     :effort (aref (roslisp:msg-slot-value last-joint-state-msg :effort)
                                   index)))))
             names)
     (roslisp:msg-slot-value
      (roslisp:msg-slot-value last-joint-state-msg :header)
      :stamp))))

(defun joint-positions (names)
  "Returns the joint positions as a list + timestamp"
  (let ((last-joint-state-msg (cpl:value *robot-joint-states-msg*)))
    (values
     (mapcar (lambda (name)
               (let ((index (position
                             name
                             (roslisp:msg-slot-value last-joint-state-msg :name)
                             :test #'string-equal)))
                 (when index
                   (aref (roslisp:msg-slot-value last-joint-state-msg :position)
                         index))))
             names)
     (roslisp:msg-slot-value
      (roslisp:msg-slot-value last-joint-state-msg :header)
      :stamp))))

(defun get-arm-joint-states (arm)
  (joint-positions (mapcar (alexandria:curry
                            #'concatenate 'string (ecase arm
                                                    (:left "l")
                                                    (:right "r")))
                           (list "_shoulder_pan_joint"
                                 "_shoulder_lift_joint"
                                 "_upper_arm_roll_joint"
                                 "_elbow_flex_joint"
                                 "_forearm_roll_joint"
                                 "_wrist_flex_joint"
                                 "_wrist_roll_joint"))))

(defun normalize-joint-angles (list-of-angles)
  (mapcar #'cl-transforms:normalize-angle list-of-angles))
