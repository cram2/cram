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

(in-package :unreal)

(defparameter *perceive-action-timeout* 3.0
  "How many seconds to wait before returning from perceive action.")

(defun make-urobosim-action-client ()
  (actionlib-client:make-simple-action-client
   'urobosim-action
   "perceive_object"
   "urobosim_msgs/PerceiveObjectAction"
   *perceive-action-timeout*))

(roslisp-utilities:register-ros-init-function make-urobosim-action-client)

(defun ensure-input-params (object-type)
  (declare (type (or string symbol) object-type))
  (when (symbolp object-type)
    (setf object-type (symbol-name object-type))))

(defun ensure-output-params (name-pose-pose-world-type-list)
  (destructuring-bind (name pose pose-world type)
      name-pose-pose-world-type-list
    (list (intern name :keyword) pose pose-world type)))

(defun make-perceive-action-goal (object-type)
  (declare (type string object-type))
  (roslisp:make-msg
   'urobosim_msgs-msg:PerceiveObjectGoal
   :type object-type))

(defun call-perceive-action (&key object-type (action-timeout *perceive-action-timeout*))
  "Returns the following list: (output-name output-pose-in-map input-object-type)"
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'urobosim-action
       :action-goal (make-perceive-action-goal (ensure-input-params object-type))
       :action-timeout action-timeout)
    (roslisp:ros-info (perceive-action) "perceive action finished.")
    (when (eq status :succeeded)
      (roslisp:with-fields ((name urobosim_msgs-msg:name)
                            (pose-in-map urobosim_msgs-msg:pose_world)
                            (pose-in-base urobosim_msgs-msg:pose))
          result
        (ensure-output-params
         (list name (cl-transforms-stamped:from-msg pose-in-base) (cl-transforms-stamped:from-msg pose-in-map) object-type))))))
