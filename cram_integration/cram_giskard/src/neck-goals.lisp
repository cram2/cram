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

(defparameter *torso-convergence-delta-joint* 0.01 "in meters")
(defparameter *cam* "ur5_wrist_3_link")

(defun make-neck-action-goal (goal-pose
                              &optional (root-link cram-tf:*robot-base-frame*))
  (declare (type cl-transforms-stamped:pose-stamped goal-pose))
  (make-giskard-goal
   :constraints (list (make-align-planes-constraint
                       root-link
                       *cam*
                       (cl-transforms-stamped:make-vector-stamped
                        cram-tf:*fixed-frame* 0.0
                        (cl-transforms:make-3d-vector 1 0 0))
                       (cl-transforms-stamped:make-vector-stamped
                        *cam* 0.0
                        (cl-transforms:make-3d-vector 0 -1 0)))
                      (make-pointing-constraint
                       root-link
                       *cam*
                       goal-pose
                       ;; (cl-transforms-stamped:make-vector-stamped
                       ;;  "rs_camera_depth_optical_frame" 0.0
                       ;;  (cl-transforms:make-3d-vector 0 0 1))
                       ))
   :cartesian-constraints (make-simple-cartesian-constraint
                           root-link *cam*
                           (cram-tf:strip-transform-stamped
                            (cram-tf:apply-transform
                             (cl-transforms-stamped:make-transform-stamped
                              cram-tf:*fixed-frame* cram-tf:*fixed-frame* 0.0
                              (cl-transforms:make-3d-vector -0.3 0.8 0.4)
                              (cl-transforms:make-identity-rotation))
                             (cram-tf:pose-stamped->transform-stamped
                              goal-pose "goal_pose"))))
   :collisions (make-avoid-all-collision)))


(defun call-neck-action (&key goal-pose action-timeout)
  (declare (type cl-transforms-stamped:pose-stamped goal-pose)
           (type (or null number) action-timeout))
  (cram-tf:visualize-marker (list goal-pose) :r-g-b-list '(0 0 1))
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'giskard-action
       :action-goal (make-neck-action-goal goal-pose)
       :action-timeout action-timeout)
    (ensure-goal-reached status)
    (values result status)
    ;; return the joint state, which is our observation
    ;; (joints:full-joint-states-as-hash-table)
    ))
