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

(in-package :cram-bullet-reasoning-belief-state)

(defvar *joint-state-pub* nil
  "Joint state publisher for the environment for RViz (and giskard collision scene)")

(defun init-joint-state-pub ()
  "Initializes *joint-state-pub* ROS publisher"
  (setf *joint-state-pub*
        (roslisp:advertise "kitchen/cram_joint_states"
                           "sensor_msgs/JointState"))
  (cpl:sleep 1.0)
  *joint-state-pub*)

(defun get-joint-state-pub ()
  (or *joint-state-pub*
      (init-joint-state-pub)))

(defun destroy-joint-state-pub ()
  (setf *joint-state-pub* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-joint-state-pub)


(defun publish-environment-joint-state (joint-state-hash-table
                                        &optional (timestamp (roslisp:ros-time)))
  (let* ((joint-state-list
           (loop for key being the hash-keys in joint-state-hash-table
                   using (hash-value value)
            collect (list key value)))
         (joint-state-msg
           (btr:make-joint-state-message
            joint-state-list :time-stamp timestamp)))
    (roslisp:publish (get-joint-state-pub) joint-state-msg)))
