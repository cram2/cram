;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :pr2-manipulation-process-module)

(defclass grasp-assignment ()
  ((pose :accessor pose :initform nil :initarg :pose)
   (side :accessor side :initform nil :initarg :side)
   (grasp-type :accessor grasp-type :initform nil :initarg :grasp-type)
   (close-radius :accessor close-radius :initform nil :initarg :close-radius)
   (handle-pair :accessor handle-pair :initform nil :initarg :handle-pair)
   (pregrasp-offset :accessor pregrasp-offset :initform nil :initarg :pregrasp-offset)
   (grasp-offset :accessor grasp-offset :initform nil :initarg :grasp-offset)
   (gripper-offset :accessor gripper-offset :initform nil :initarg :gripper-offset)
   (ik-cost :accessor ik-cost :initform nil :initarg :ik-cost)
   (object-part :accessor object-part :initform nil :initarg :object-part)))

(defclass arm-pose ()
  ((joint-values :accessor joint-values :initform nil :initarg :joint-values)
   (joint-names :accessor joint-names :initform nil :initarg :joint-names)
   (arm-side :accessor arm-side :initform nil :initarg :arm-side)
   (name :accessor name :initform nil :initarg :name)))

(define-condition move-arm-no-ik-solution (manipulation-failure) ())
(define-condition move-arm-ik-link-in-collision (manipulation-failure) ())

(defvar *gripper-action-left* nil)
(defvar *gripper-action-right* nil)
(defvar *gripper-grab-action-left* nil)
(defvar *gripper-grab-action-right* nil)

;; TODO: are these variables used anywhere? They're not used in this package, nor exported.
(defvar *trajectory-action-left* nil)
(defvar *trajectory-action-right* nil)
(defvar *trajectory-action-both* nil)
(defvar *trajectory-action-torso* nil)

;; (defvar *left-safe-pose* (make-pose-stamped
;;                           "base_link" (ros-time)
;;                           (cl-transforms:make-3d-vector 0.3 0.5 1.3)
;;                           (cl-transforms:euler->quaternion :ax pi)))
;; (defvar *right-safe-pose* (make-pose-stamped
;;                            "base_link" (ros-time)
;;                            (cl-transforms:make-3d-vector 0.3 -0.5 1.3)
;;                            (cl-transforms:euler->quaternion :ax pi)))

(defun init-pr2-manipulation-process-module ()
  ;; TODO: make this robot-agnostic, so we could reuse this package (which actually contains generic low-level manipulation plans) for other robots too.
  (setf *gripper-action-left*
        (actionlib:make-action-client
         "/l_gripper_controller/gripper_action"
         "pr2_controllers_msgs/Pr2GripperCommandAction"))
  (setf *gripper-action-right*
        (actionlib:make-action-client
         "/r_gripper_controller/gripper_action"
         "pr2_controllers_msgs/Pr2GripperCommandAction"))
  ;; TODO: apart from here, *trajectory-action-torso* isn't used anywhere else in the package, nor is it exported.
  (setf *trajectory-action-torso*
        (actionlib:make-action-client
         "/torso_controller/joint_trajectory_action"
         "pr2_controllers_msgs/JointTrajectoryAction")))

(roslisp-utilities:register-ros-init-function
 init-pr2-manipulation-process-module)

(defun execute-goal (server goal)
  (multiple-value-bind (result status)
      (actionlib:call-goal server goal)
    (unless (eq status :succeeded)
      (cpl-impl:fail 'manipulation-failed
                     :format-control "Manipulation failed"))
    result))

(defun register-arm-pose (arm-pose)
  (push arm-pose *registered-arm-poses*))

(defun register-arm-pose-values (name side joint-names joint-values)
  (register-arm-pose (make-instance
                      'arm-pose
                      :arm-side side
                      :joint-names joint-names
                      :joint-values joint-values
                      :name name)))

(defun register-current-arm-pose (name side)
  ;; TODO: replace this with a prolog call (which would take the joint names to be what's appropriate for the used robot, whatever it is)
  (let* ((joints (cut:var-value '?joints
                                (first (prolog:prolog
                                         `(and (robot ?robot)
                                               (arm-joints ?robot ,side ?joints))))))
         (joint-values (mapcar (lambda (joint)
                                 (moveit:get-joint-value joint))
                               joints)))
    (register-arm-pose-values name side joints joint-values)))

(defun registered-arm-pose (name side)
  (find name *registered-arm-poses*
        :test (lambda (name arm-pose)
                (and (eql name (name arm-pose))
                     (eql side (arm-side arm-pose))))))

(defun links-for-arm-side (side)
  (cut:var-value '?links
                 (first (prolog:prolog
                          `(and (robot ?robot)
                                (arm-links ?robot ,side ?links))))))

(defun execute-move-arm-poses (side poses-stamped goal-spec
                               &key allowed-collision-objects
                                 ignore-collisions
                                 plan-only
                                 start-state
                                 collidable-objects
                                 max-tilt)
  ;; TODO: this function is not exported nor called from anywhere inside the package. Keep?
  (ros-info (pr2 manip-pm) "Executing multi-pose arm movement")
  (let* ((updated-goal-spec (mot-man:enriched-goal-specification goal-spec
                                                                 :keys `((:allowed-collision-objects ,allowed-collision-objects)
                                                                         (:ignore-collisions ,ignore-collisions)
                                                                         (:collidable-objects ,collidable-objects)
                                                                         (:max-tilt ,max-tilt)
                                                                         (:plan-only ,plan-only)
                                                                         (:start-state ,start-state))
                                                                 :arm-pose-goals `((,side ,poses-stamped)))))
    (cond (plan-only (mot-man:trajectories (mot-man:execute-arm-action updated-goal-spec)))
          (t (mot-man:execute-arm-action updated-goal-spec)))))

(defun gripper-at-pose-p (side pose-stamped
                          &key cartesian-thres angular-thres)
  (let* ((cartesian-distance-threshold (or cartesian-thres 0.02))
         (angular-distance-threshold (or angular-thres 0.1))
         (link-name (link-name side))
         (link-id-pose
           (cl-transforms-stamped:pose->pose-stamped
            link-name 0.0 (cl-transforms:make-identity-pose)))
         (link-compare-pose
           (cl-transforms-stamped:transform-pose-stamped
            *transformer*
            :pose link-id-pose
            :target-frame (frame-id pose-stamped)
            :timeout *tf-default-timeout*))
         (dist-v (v-dist
                  (origin pose-stamped)
                  (origin link-compare-pose)))
         (dist-a-pre (angle-between-quaternions
                      (cl-transforms:orientation pose-stamped)
                      (cl-transforms:orientation link-compare-pose)))
         (dist-a (cond ((> dist-a-pre pi)
                        (- dist-a-pre (* 2 pi)))
                       (t dist-a-pre))))
    (and (<= (abs dist-v) cartesian-distance-threshold)
         (<= (abs dist-a) angular-distance-threshold))))

(define-hook cram-language::on-close-gripper (side max-effort position))
(define-hook cram-language::on-open-gripper (side max-effort position))

(defun close-gripper (side &key (max-effort 100.0) (position 0.0))
  (cram-language::on-close-gripper side max-effort position)
  (let ((client (ecase side
                  (:right *gripper-action-right*)
                  (:left *gripper-action-left*))))
    (unwind-protect
         (actionlib:send-goal-and-wait
          client (actionlib:make-action-goal client
                   (position command) position
                   (max_effort command) max-effort)
          :result-timeout 1.0)
      (cram-occasions-events:on-event
       (make-instance
        'cram-plan-occasions-events:robot-state-changed
        :timestamp 0.0)))))

(defun open-gripper (side &key (max-effort 100.0) (position 0.085))
  (cram-language::on-open-gripper side max-effort position)
  (let ((client (ecase side
                  (:right *gripper-action-right*)
                  (:left *gripper-action-left*))))
    (prog1
        (unwind-protect
             (actionlib:send-goal-and-wait
              client (actionlib:make-action-goal client
                       (position command) position
                       (max_effort command) max-effort)
              :result-timeout 1.0)
          (cram-occasions-events:on-event
           (make-instance 'cram-plan-occasions-events:robot-state-changed
                          :timestamp 0.0)))
      (with-vars-bound (?carried-object ?gripper-link)
          (lazy-car (prolog `(and (object-in-hand ?carried-object ,side)
                                  (robot ?robot)
                                  (end-effector-link ?robot ,side ?gripper-link))))
        (unless (is-var ?carried-object)
          (cram-occasions-events:on-event
           (make-instance 'cram-plan-occasions-events:object-detached
             :object ?carried-object
             :link ?gripper-link
             :side side)))))))

(defclass manipulated-perceived-object (desig:object-designator-data) ())

(defun update-object-designator-pose (object-designator new-object-pose)
  (equate object-designator
          (desig:make-effective-designator
           object-designator :data-object (make-instance 'manipulated-perceived-object
                                                         :pose new-object-pose))))

(def-process-module pr2-manipulation-process-module (desig)
  (collision-environment-set-laser-period)
  (apply #'call-action (reference desig)))

(defun update-grasped-object-designator (obj grippers &key new-properties)
  (let* ((target-frame (var-value '?target-frame
                                  (lazy-car
                                   (prolog:prolog
                                    `(and (robot ?robot)
                                          (cram-robot-interfaces:end-effector-link
                                           ?robot
                                           ,(car grippers)
                                           ?target-frame))))))
         (obj-pose-in-gripper (pose->pose-stamped
                               target-frame
                               0.0
                               (cl-transforms-stamped:transform-pose-stamped
                                *transformer*
                                :pose (obj-desig-location (current-desig obj))
                                :target-frame target-frame
                                :timeout *tf-default-timeout*)))
         (loc-desig-in-gripper (make-designator
                                :location
                                (append `((:pose ,obj-pose-in-gripper)
                                          (:in :gripper))
                                        (mapcar (lambda (grip)
                                                  `(:gripper ,grip))
                                                grippers)))))
    (make-designator
     :object
     (append `((:at ,loc-desig-in-gripper) . ,(remove :at (description obj) :key #'car))
             new-properties)
     obj)))

(defun update-picked-up-object-designator (obj-desig gripper side height)
  "Function that creates and equates a new obj-designator to an object
that has been grasped. `gripper' shall either include the symbols
GRIPPER or BOTH-GRIPPERS to discriminate between
single and dual grasps. `Side' indicates with respect to which gripper
the new location designator shall be constructed. `height' is the
difference in z-coordinate of the grasping point of the object and
its' supporting plane."
  (style-warn 'simple-style-warning
              :format-control "Use of deprecated form
              UPDATE-PICKED-UP-OBJECT-DESIGNATOR. Please use
              UPDATE-GRASPED-OBJECT-DESIGNATOR instead.")
  ;; get current pose of the object in map frame
  (let* ((obj-pose (cl-transforms-stamped:transform-pose-stamped
                    *transformer*
                    :pose (obj-desig-location (current-desig obj-desig))
                    :target-frame *fixed-frame*
                    :timeout *tf-default-timeout*))
         ;; build a new location designator for the object:
         ;; the transform will be in the wrist frame of the `side' gripper
         ;; thus it'll move with the gripper;
         ;; adding (in ,`gripper') indicates whether it has been grasped
         ;; with one or two grippers;
         ;; (orientation ...) is the intended put down orientation of the object;
         (new-loc-desig
           (make-designator
            :location
            `((:in ,gripper)
              (:side ,side)
              (:pose ,(copy-pose-stamped
                      (cl-transforms-stamped:transform-pose-stamped
                       *transformer*
                       :pose obj-pose
                       :target-frame (link-name side)
                       :timeout *tf-default-timeout*)
                      :stamp 0.0))
              (:height ,height)
              (:orientation ,(cl-transforms:orientation obj-pose))))))
    ;; build and equate new object designator using the new location designator
    ;; NOTE: this usage of make-designator does it both in one line
    (make-designator
     :object
     `((:at ,new-loc-desig) . ,(remove :at (description obj-desig) :key #'car))
     obj-desig)))
