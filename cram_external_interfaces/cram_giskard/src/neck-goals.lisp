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

(defparameter *neck-convergence-delta-xy* 0.01 "in meters")
(defparameter *neck-convergence-delta-joint* 0.1 "In radians, about 6 degrees")
(defparameter *cam* "ur5_wrist_3_link")
(defparameter *donbot-camera-offset* (cl-transforms:make-transform
                                      (cl-transforms:make-3d-vector -0.3 0.8 0.4)
                                      (cl-transforms:make-identity-rotation)))

(defun make-neck-action-goal (goal-pose
                              &key
                                (root-link cram-tf:*robot-base-frame*)
                                camera-link
                                camera-offset)
  (declare (type cl-transforms-stamped:pose-stamped goal-pose))
  (let ((neck-joints (first (get-neck-joint-names-and-positions-list))))
    (make-giskard-goal
     :constraints `(,(make-pointing-constraint
                      root-link
                      camera-link
                      goal-pose)
                    ;; for PTU heads align planes doesn't make sense
                    ,@(when (> (length neck-joints) 2)
                        `((make-align-planes-constraint
                           root-link
                           camera-link
                           (cl-transforms-stamped:make-vector-stamped
                            cram-tf:*fixed-frame* 0.0
                            (cl-transforms:make-3d-vector 1 0 0))
                           (cl-transforms-stamped:make-vector-stamped
                            camera-link 0.0
                            (cl-transforms:make-3d-vector 0 -1 0)))))
                    ;; (cl-transforms-stamped:make-vector-stamped
                    ;;  "rs_camera_depth_optical_frame" 0.0
                    ;;  (cl-transforms:make-3d-vector 0 0 1))
                    )
     :cartesian-constraints (when camera-offset
                              (make-cartesian-constraint
                               root-link camera-link
                               (cram-tf:strip-transform-stamped
                                (cram-tf:apply-transform
                                 (cl-transforms-stamped:transform->transform-stamped
                                  cram-tf:*fixed-frame* cram-tf:*fixed-frame* 0.0
                                  *donbot-camera-offset*)
                                 (cram-tf:pose-stamped->transform-stamped
                                  goal-pose "goal_pose")))))
     :collisions (make-avoid-all-collision))))

(defun make-neck-joint-action-goal (joint-state)
  (declare (type list joint-state))
  (make-giskard-goal
   :joint-constraints (make-joint-constraint joint-state )
   :collisions (make-avoid-all-collision)))

(defun ensure-neck-goal-input ()
  (if (eq (rob-int:get-robot-name) :iai-donbot)
      (list *cam* *donbot-camera-offset*)
      (let ((camera-frame
              (cut:var-value
               '?frame
               (car (prolog:prolog `(and (rob-int:robot ?robot)
                                         (rob-int:camera-frame ?robot ?frame)))))))
        (when (cut:is-var camera-frame)
          (error "[giskard] Robot camera frame was not defined."))
        (list camera-frame))))

(defun ensure-neck-joint-goal-input (goal-configuration)
  (let ((neck-joints (first (get-neck-joint-names-and-positions-list))))
    (if (and (listp goal-configuration)
             (= (length goal-configuration)
                (length neck-joints)))
        (get-neck-joint-names-and-positions-list goal-configuration)
        (progn (roslisp:ros-warn (giskard neck)
                                 "Joint goal ~a was not a list of ~a. Ignoring."
                                 goal-configuration
                                 (length neck-joints))
               (get-neck-joint-names-and-positions-list)))))

(defun ensure-neck-joint-goal-reached (goal-configuration)
  (when goal-configuration
    (let ((current-angles
            (cram-tf:normalize-joint-angles
             (second (get-neck-joint-names-and-positions-list))))
          (goal-angles
            (cram-tf:normalize-joint-angles
             (mapcar #'second goal-configuration))))
      (unless (cram-tf:values-converged
               current-angles goal-angles *neck-convergence-delta-joint*)
        (make-instance 'common-fail:manipulation-goal-not-reached
          :description (format nil "Giskard did not converge to goal:~%~
                                    Neck current angles: ~a~%~
                                    Neck goal angles: ~a~%~
                                    Convergence delta: ~a. "
                               current-angles goal-angles
                               *neck-convergence-delta-joint*))))))

(defun call-neck-action (&key action-timeout goal-pose)
  (declare (type (or number null) action-timeout)
           (type cl-transforms-stamped:pose-stamped goal-pose))

  (let* ((camera-frame-and-offset (ensure-neck-goal-input))
         (camera-frame (first camera-frame-and-offset))
         (camera-offset (second camera-frame-and-offset)))

    (cram-tf:visualize-marker (list goal-pose) :r-g-b-list '(0 0 1))
    (call-action
     :action-goal (make-neck-action-goal goal-pose
                                         :camera-link camera-frame
                                         :camera-offset camera-offset)
     :action-timeout action-timeout)))

(defun call-neck-joint-action (&key action-timeout goal-configuration)
  (declare (type (or number null) action-timeout)
           (type (or list null) goal-configuration))

  (let ((joint-state
          (ensure-neck-joint-goal-input goal-configuration)))

    (call-action
     :action-goal (make-neck-joint-action-goal joint-state)
     :action-timeout action-timeout
     :check-goal-function (lambda (result status)
                            (declare (ignore result status))
                            (ensure-neck-joint-goal-reached goal-configuration)))))
