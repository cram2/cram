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

(defvar *giskard-action-client* nil)

(defparameter *giskard-action-timeout* 30.0
  "How many seconds to wait before returning from giskard action.")
(defparameter *giskard-convergence-delta-xy* 0.005 "in meters")
(defparameter *giskard-convergence-delta-theta* 0.1 "in radiants, about 6 degrees")
(defparameter *giskard-convergence-delta-joint* 0.17 "in radiants, about 10 degrees")

(defvar *left-tool-frame* nil "Tool frame of the left arm. Initialized from pr2-description.")
(defvar *right-tool-frame* nil "Tool frame of the right arm. Initialized from pr2-desc.")

(defun init-giskard-action-client ()
  (setf *giskard-action-client* (actionlib:make-action-client
                                 "qp_controller/command"
                                 "giskard_msgs/WholeBodyAction"))
  (loop until (actionlib:wait-for-server *giskard-action-client* 5.0)
        do (roslisp:ros-info (giskard-action-client) "Waiting for giskard action server..."))
  (roslisp:ros-info (giskard-action-client) "giskard action client created.")
  *giskard-action-client*)

(defun destroy-giskard-action-client ()
  (setf *giskard-action-client* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-giskard-action-client)

(defun get-giskard-action-client ()
  (or *giskard-action-client*
      (init-giskard-action-client)))

(defun init-giskard-tool-frames ()
  (cut:with-vars-bound (?left-tool-frame ?right-tool-frame)
      (cut:lazy-car
       (prolog:prolog
        `(and (cram-robot-interfaces:robot ?robot)
              (cram-robot-interfaces:robot-tool-frame ?robot :left ?left-tool-frame)
              (cram-robot-interfaces:robot-tool-frame ?robot :right ?right-tool-frame))))
    (if (cut:is-var ?left-tool-frame)
        (roslisp:ros-info (low-level giskard-init)
                          "?left-tool-frame is unknown.
Did you load a robot description package?")
        (progn
          (setf *left-tool-frame* ?left-tool-frame)
          (roslisp:ros-info (low-level giskard-init)
                            "Set *left-tool-frame* to ~s."  ?left-tool-frame)))
    (if (cut:is-var ?right-tool-frame)
        (roslisp:ros-info (low-level giskard-init)
                          "?right-tool-frame is unknown.
Did you load a robot description package?")
        (progn
          (setf *right-tool-frame* ?right-tool-frame)
          (roslisp:ros-info (low-level giskard-init)
                            "Set *right-tool-frame* to ~s."  ?right-tool-frame)))))

(roslisp-utilities:register-ros-init-function init-giskard-tool-frames)
