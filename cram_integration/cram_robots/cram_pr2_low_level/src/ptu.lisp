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

(defparameter *ptu-action-timeout* 5.0
  "How many seconds to wait before returning from PTU action.")

(defvar *ptu-action-client* nil)

(defun init-ptu-action-client ()
  (setf *ptu-action-client* (actionlib:make-action-client
                             "head_traj_controller/point_head_action"
                             "pr2_controllers_msgs/PointHeadAction"))
  (loop until (actionlib:wait-for-server *ptu-action-client* 5.0)
        do (roslisp:ros-info (ptu-action-client) "Waiting for PTU action server..."))
  (roslisp:ros-info (ptu-action-client) "PTU action client created.")
  *ptu-action-client*)

(defun destroy-ptu-action-client ()
  (setf *ptu-action-client* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-ptu-action-client)

(defun get-ptu-action-client ()
  (or *ptu-action-client*
      (init-ptu-action-client)))

(defun make-ptu-action-goal (point-stamped)
  (actionlib:make-action-goal
      (get-ptu-action-client)
    max_velocity 1
    min_duration 0.3
    pointing_frame "high_def_frame"
    (x pointing_axis) 1.0
    (y pointing_axis) 0.0
    (z pointing_axis) 0.0
    target (cl-transforms-stamped:to-msg point-stamped)))

(defun ensure-ptu-input-parameters (frame point direction)
  (declare (type (or null string) frame)
           (type (or cl-transforms:3d-vector
                     cl-transforms:point cl-transforms-stamped:point-stamped
                     cl-transforms:pose cl-transforms-stamped:pose-stamped))
           (type (or null symbol) direction))
  "Returns a point-stamped in `frame'"
  (let ((frame (or (when (typep point 'cl-transforms-stamped:pose-stamped)
                     (cl-transforms-stamped:frame-id point))
                   frame
                   cram-tf:*robot-base-frame*)))
    (if direction
        (cl-transforms-stamped:make-point-stamped
         cram-tf:*robot-base-frame*
         0.0
         (cl-transforms:make-3d-vector 10.0 0.0 1.5))
        (cram-tf:ensure-point-in-frame
         (etypecase point
           (cl-transforms:point ; also covers 3d-vector and point-stamped
            point)
           (cl-transforms:pose          ; also covers pose-stamped
            (cl-transforms:origin point)))
         frame))))

(defun ensure-ptu-goal-reached (status)
  (when (eql status :timeout)
    (roslisp:ros-warn (pr2-ll ptu) "PTU action timed out."))
  (unless (eql status :succeeded)
    (cpl:fail 'common-fail:ptu-low-level-failure :description "PTU action did not succeed."))
  ;; todo: would be nice to check if the point is actually visible from the
  ;; end configuration
  ;; using looking-at or point-head-at from cram_3d_world
  )

(defun call-ptu-action (&key (frame cram-tf:*robot-base-frame*)
                          (pose (cl-transforms:make-identity-pose))
                          direction
                          (action-timeout *ptu-action-timeout*))
  (declare (type (or null string) frame)
           (type (or cl-transforms:3d-vector
                     cl-transforms:point cl-transforms-stamped:point-stamped
                     cl-transforms:pose cl-transforms-stamped:pose-stamped)))
  (let ((goal-point-stamped (ensure-ptu-input-parameters frame pose direction)))
    (multiple-value-bind (result status)
        (cpl:with-failure-handling
            ((simple-error (e)
               (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
               (init-ptu-action-client)
               (cpl:retry)))
          (let ((actionlib:*action-server-timeout* 10.0))
            (actionlib:call-goal
             (get-ptu-action-client)
             (make-ptu-action-goal goal-point-stamped)
             :timeout action-timeout)))
      (ensure-ptu-goal-reached status)
      (values result status))))

(defun shake-head (n-times)
  (dotimes (n n-times)
    (call-ptu-action :pose (cl-transforms:make-3d-vector 5.0 1.0 1.2))
    (call-ptu-action :pose (cl-transforms:make-3d-vector 5.0 -1.0 1.2))))

(defun look-at-gripper (left-or-right)
  (call-ptu-action :frame (ecase left-or-right
                            (:left "l_gripper_tool_frame")
                            (:right "r_gripper_tool_frame"))))
