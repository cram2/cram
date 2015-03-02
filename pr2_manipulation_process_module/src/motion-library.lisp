;;; Copyright (c) 2012, Jan Winkler <winkler@informatik.uni-bremen.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
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

(in-package :pr2-manipulation-process-module)

(defvar *registered-arm-poses* nil)

(defclass manipulation-parameters ()
  ((arm :accessor arm :initform nil :initarg :arm)
   (safe-pose :accessor safe-pose :initform nil :initarg :safe-pose)
   (grasp-type :accessor grasp-type :initform nil :initarg :grasp-type)
   (object-part :accessor object-part :initform nil :initarg :object-part)
   (max-collisions-tolerance :accessor max-collisions-tolerance :initform nil :initarg :max-collisions-tolerance)))

(defclass grasp-parameters (manipulation-parameters)
  ((grasp-pose :accessor grasp-pose :initform nil :initarg :grasp-pose)
   (pregrasp-pose :accessor pregrasp-pose :initform nil :initarg :pregrasp-pose)
   (effort :accessor effort :initform nil :initarg :effort)
   (close-radius :accessor close-radius :initform nil :initarg :close-radius)))

(defclass putdown-parameters (manipulation-parameters)
  ((pre-putdown-pose :accessor pre-putdown-pose :initform nil :initarg :pre-putdown-pose)
   (putdown-pose :accessor putdown-pose :initform nil :initarg :putdown-pose)
   (unhand-pose :accessor unhand-pose :initform nil :initarg :unhand-pose)
   (open-radius :accessor open-radius :initform nil :initarg :open-radius)))

(defclass park-parameters (manipulation-parameters)
  ((park-pose :accessor park-pose :initform nil :initarg :park-pose)))

(define-hook on-execute-grasp-with-effort (object-name))
(define-hook on-execute-grasp-gripper-closed
    (object-name gripper-effort gripper-close-pos side pregrasp-pose safe-pose))
(define-hook on-execute-grasp-gripper-positioned-for-grasp
    (object-name gripper-effort gripper-close-pos side pregrasp-pose safe-pose))
(define-hook on-execute-grasp-pregrasp-reached
    (object-name gripper-effort gripper-close-pos side pregrasp-pose safe-pose))

(defun joint-trajectory-point->joint-state (header joint-names trajectory-point)
  (roslisp:with-fields (positions) trajectory-point
    (roslisp:make-message
     "sensor_msgs/JointState"
     :header header
     :name joint-names
     :position positions)))

(defun arm-pose->trajectory (arm pose &key ignore-collisions)
  "Calculated a trajectory from the current pose of gripper `arm' when
trying to assume the pose `pose'."
  (cpl:with-failure-handling
      (((or cram-plan-failures::manipulation-failure
            moveit::moveit-failure) (f)
         (declare (ignore f))
         (ros-error
          (pr2 grasp)
          "Failed to generate trajectory for ~a."
          arm)))
    (execute-move-arm-pose arm pose :quiet t :plan-only t
                                    :ignore-collisions ignore-collisions)))

(defun link-distance-from-pose (link-name pose-stamped)
  (let* ((link-identity-pose (cl-tf-datatypes:pose->pose-stamped
                              link-name 0.0
                              (cl-transforms:make-identity-pose)))
         (link-in-pose-frame (cl-tf2:transform-pose
                              *tf2-buffer*
                              :pose link-identity-pose
                              :target-frame (cl-tf-datatypes:frame-id pose-stamped)
                              :timeout cram-roslisp-common:*tf-default-timeout*
                              :use-current-ros-time t)))
    (cl-transforms:v-dist (cl-transforms:origin link-in-pose-frame)
                          (cl-transforms:origin pose-stamped))))

(defun pose-assumed (parameter-sets slot-name &key (threshold 3.0))
  "Checks whether the pose defined in the slot `slot-name' was assumes for all parameter sets in `parameter-sets'. The value `threshold' is used as the maximum cartesian distance by which the to be assumed and the actual pose might differ in order to be valid."
  (every #'identity
         (mapcar (lambda (parameter-set)
                   (let* ((link-name (cut:var-value
                                      '?link
                                      (first
                                       (crs:prolog
                                        `(manipulator-link
                                          ,(arm parameter-set) ?link)))))
                          (pose-stamped (slot-value parameter-set slot-name))
                          (distance (link-distance-from-pose link-name pose-stamped)))
                     (<= distance threshold)))
                 parameter-sets)))

(defun assume-poses (parameter-sets slot-name &key ignore-collisions)
  "Moves all arms defined in `parameter-sets' into the poses given by
the slot `slot-name' as defined in the respective parameter-sets. If
`ignore-collisions' is set, all collisions are ignored during the
motion."
  (ros-info (pr2 motion) "Assuming ~a '~a' pose~a"
            (length parameter-sets) slot-name
            (cond ((= (length parameter-sets) 1) "")
                  (t "s")))
  (let ((max-collisions-tolerance
          (loop for parameter-set in parameter-sets
                as max-collisions-tolerance = (max-collisions-tolerance parameter-set)
                when max-collisions-tolerance
                  maximizing max-collisions-tolerance)))
    (cpl:with-failure-handling
        ((cram-plan-failures:manipulation-failure (f)
           (declare (ignore f))
           (when (and max-collisions-tolerance
                      (> max-collisions-tolerance 0))
             (decf max-collisions-tolerance)
             (when (< max-collisions-tolerance 1)
               (setf max-collisions-tolerance nil)
               (setf ignore-collisions t))
             (cpl:retry))))
      (cond ((= (length parameter-sets) 1)
             (when (slot-value (first parameter-sets) slot-name)
               (execute-move-arm-pose
                (arm (first parameter-sets))
                (slot-value (first parameter-sets) slot-name)
                :ignore-collisions ignore-collisions)))
            (t (moveit:execute-trajectories
                (cpl:mapcar-clean
                 (lambda (parameter-set)
                   (when (slot-value parameter-set slot-name)
                     (arm-pose->trajectory
                      (arm parameter-set)
                      (slot-value parameter-set slot-name)
                      :ignore-collisions ignore-collisions)))
                 parameter-sets)
                :ignore-va t)))
      (unless (pose-assumed parameter-sets slot-name)
        (ros-warn (pr2 manip-pm) "Failed to assume at least one pose (distance too large).")
        (cpl:fail 'cram-plan-failures:manipulation-failure)))))

(defmacro with-parameter-sets (parameter-sets &body body)
  "Defines parameter-set specific functions (like assuming poses) for
the manipulation parameter sets `parameter-sets' and executes the code
`body' in this environment."
  `(labels ((assume (pose-slot-name &optional ignore-collisions)
              (assume-poses
               ,parameter-sets pose-slot-name
               :ignore-collisions ignore-collisions)))
     ,@body))

(defun link-name (arm)
  "Returns the TF link name associated with the wrist of the robot's
arm `arm'."
  (cut:var-value '?link (first (crs:prolog `(manipulator-link ,arm ?link)))))

(defun execute-parks (parameter-sets)
  (with-parameter-sets parameter-sets
    (assume 'park-pose)))

(defun open-gripper-if-necessary (arm &key (threshold 0.08))
  "Opens the gripper on the robot's arm `arm' if its current position
is smaller than `threshold'."
  (when (< (get-gripper-state arm) threshold)
    (open-gripper arm)))

(defun gripper-closed-p (arm &key (threshold 0.0025))
  "Returns `t' when the robot's gripper on the arm `arm' is smaller
than `threshold'."
  (< (get-gripper-state arm) threshold))

(defun execute-grasps (object-name parameter-sets)
  "Executes simultaneous grasping of the object given by the name
`object-name'. The grasps (object-relative gripper positions,
grasp-type, effort to use) are defined in the list `parameter-sets'."
  (with-parameter-sets parameter-sets
    (cpl:with-failure-handling
        (((or cram-plan-failures:manipulation-failure
              cram-plan-failures:object-lost) (f)
           (declare (ignore f))
           (ros-warn (pr2 manip-pm) "Falling back to safe pose")
           (assume 'safe-pose t)))
      (assume 'pregrasp-pose)
      (cpl:par-loop (parameter-set parameter-sets)
        (open-gripper-if-necessary (arm parameter-set)))
      (cpl:with-failure-handling
          (((or cram-plan-failures:manipulation-failure
                cram-plan-failures:object-lost) (f)
             (declare (ignore f))
             (ros-warn (pr2 manip-pm) "Falling back to pregrasp pose")
             (assume 'pregrasp-pose)))
        (moveit:without-collision-object object-name
          (assume 'grasp-pose t)
          (cpl:par-loop (parameter-set parameter-sets)
            (close-gripper (arm parameter-set) :max-effort (effort parameter-set)))
          (unless (every #'not (mapcar
                                (lambda (parameter-set)
                                  (gripper-closed-p (arm parameter-set)))
                                parameter-sets))
            (ros-warn (pr2 manip-pm) "At least one gripper failed to grasp the object")
            (cpl:par-loop (parameter-set parameter-sets)
              (open-gripper-if-necessary (arm parameter-set)))
            (cpl:fail 'cram-plan-failures:object-lost)))
        (dolist (parameter-set parameter-sets)
          (moveit:attach-collision-object-to-link
           object-name (link-name (arm parameter-set))))))))

(defun execute-putdowns (object-name parameter-sets)
  "Executes simultaneous putting down of the object in hand given by
the name `object-name'. The current grasps (object-relative gripper
positions, grasp-type, effort to use) are defined in the list
`parameter-sets'."
  (with-parameter-sets parameter-sets
    (cpl:with-failure-handling
        ((cram-plan-failures:manipulation-failure (f)
           (declare (ignore f))
           (assume 'safe-pose t)))
      (assume 'pre-putdown-pose)
      (cpl:with-failure-handling
          ((cram-plan-failures:manipulation-failure (f)
             (declare (ignore f))
             (assume 'pre-putdown-pose)))
        (assume 'putdown-pose t)
        (cpl:par-loop (param-set parameter-sets)
          (open-gripper (arm param-set)))
        (block unhand
          (cpl:with-failure-handling
              ((cram-plan-failures:manipulation-failure (f)
                 (declare (ignore f))
                 (assume 'safe-pose t)
                 (return-from unhand)))
            (assume 'unhand-pose))))))
  (dolist (param-set parameter-sets)
    (moveit:detach-collision-object-from-link
     object-name (link-name (arm param-set)))))

(defun execute-lift (grasp-assignments distance)
  (let ((target-arm-poses
          (mapcar (lambda (grasp-assignment)
                    (cons (side grasp-assignment)
                          (let ((pose-straight
                                  (cl-tf2:transform-pose
                                   *tf2-buffer*
                                   :pose (cl-tf-datatypes:pose->pose-stamped
                                          (link-name (side grasp-assignment))
                                          0.0
                                          (cl-transforms:make-identity-pose))
                                   :target-frame "base_link"
                                   :timeout cram-roslisp-common:*tf-default-timeout*)))
                            (cl-tf-datatypes:copy-pose-stamped
                             pose-straight
                             :origin (cl-transforms:v+
                                      (cl-transforms:origin pose-straight)
                                      (cl-transforms:make-3d-vector 0 0 distance))))))
                  grasp-assignments)))
    (cond ((= (length grasp-assignments) 1)
           (destructuring-bind (arm . pose) (first target-arm-poses)
             (execute-move-arm-pose arm pose :ignore-collisions t)))
          (t
           (moveit:execute-trajectories
            (mapcar (lambda (target-arm-pose)
                      (destructuring-bind (arm . pose)
                          target-arm-pose
                        (execute-move-arm-pose
                         arm pose :plan-only t
                                  :quiet t
                                  :ignore-collisions t)))
                    target-arm-poses))))))

(defun relative-pose (pose pose-offset)
  "Applies the pose `pose-offset' as transformation into the pose
`pose' and returns the result in the frame of `pose'."
  (cl-tf-datatypes:pose->pose-stamped
   (cl-tf-datatypes:frame-id pose)
   (ros-time)
   (cl-transforms:transform-pose
    (cl-transforms:pose->transform pose)
    pose-offset)))
