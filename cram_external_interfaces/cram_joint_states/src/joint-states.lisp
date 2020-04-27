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

(in-package :joints)

(defvar *joint-state-sub* nil
  "Subscriber for robot's joint state topic.")

(defparameter *joint-state-frequency* 10.0d0
  "How often to update the fluent in Hz")
(defvar *joint-state-timestamp* 0.0d0
  "Timestamp of the last fluent update in secs.")

(defvar *robot-joint-states-msg* (cpl:make-fluent :name :robot-joint-states)
  "ROS message containing robot's current joint states.")

(defun init-joint-state-sub ()
  "Initializes *joint-state-sub*,
updating `*robot-joint-states-msg*' with frequency given in `*joint-state-frequency*'."
  (let ((update-every-?-secs (the double-float (/ 1.0d0 *joint-state-frequency*))))
    (declare (double-float update-every-?-secs))
    (flet ((joint-state-sub-cb (joint-state-msg)
             (when (> (the double-float (- (roslisp:ros-time) *joint-state-timestamp*))
                      update-every-?-secs)
               (setf *joint-state-timestamp* (the double-float (roslisp:ros-time)))
               (setf (cpl:value *robot-joint-states-msg*) joint-state-msg))))
      (setf *joint-state-sub*
            (roslisp:subscribe "joint_states"
                               "sensor_msgs/JointState"
                               #'joint-state-sub-cb)))))

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

(defun joint-positions (names &optional state-fluent)
  "Returns the joint positions as a list + timestamp"
  (let ((last-joint-state-msg (cpl:value (or state-fluent *robot-joint-states-msg*))))
    (when last-joint-state-msg
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
        :stamp)))))

(defun joint-velocities (names &optional state-fluent)
  "Returns the joint velocities as a list + timestamp"
  (let ((last-joint-state-msg (cpl:value (or state-fluent *robot-joint-states-msg*))))
    (values
     (mapcar (lambda (name)
               (let ((index (position
                             name
                             (roslisp:msg-slot-value last-joint-state-msg :name)
                             :test #'string-equal)))
                 (when index
                   (aref (roslisp:msg-slot-value last-joint-state-msg :velocity)
                         index))))
             names)
     (roslisp:msg-slot-value
      (roslisp:msg-slot-value last-joint-state-msg :header)
      :stamp))))

(defun normalize-joint-angles (list-of-angles)
  (mapcar #'cl-transforms:normalize-angle list-of-angles))


(defun full-joint-states-as-hash-table (&optional state-fluent)
  (let ((last-joint-state-msg (cpl:value (or state-fluent *robot-joint-states-msg*))))
    (when last-joint-state-msg
      (let ((result-hash-table (make-hash-table :test 'equal)))
        (map 'list
             (lambda (name position)
               (setf (gethash name result-hash-table) position))
             (roslisp:msg-slot-value last-joint-state-msg :name)
             (roslisp:msg-slot-value last-joint-state-msg :position))
        ;; hpn needs the odom joints
        ;; perhaps CRAM will work with odom joints as well one day
        (let* ((robot-pose (cram-tf:robot-current-pose))
               (robot-x (cl-transforms:x (cl-transforms:origin robot-pose)))
               (robot-y (cl-transforms:y (cl-transforms:origin robot-pose))))
             (multiple-value-bind (axis angle)
                 (cl-transforms:quaternion->axis-angle
                  (cl-transforms:orientation robot-pose))
               (when (< (cl-transforms:z axis) 0)
                 (setf angle (- angle)))
               (setf (gethash "odom_x_joint" result-hash-table) robot-x)
               (setf (gethash "odom_y_joint" result-hash-table) robot-y)
               (setf (gethash "odom_z_joint" result-hash-table) angle)
               result-hash-table))))))
