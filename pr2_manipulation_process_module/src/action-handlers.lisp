;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of University of Bremen nor the names of its
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

(defclass grasp-parameters ()
  ((grasp-pose :accessor grasp-pose :initform nil :initarg :grasp-pose)
   (pregrasp-pose :accessor pregrasp-pose :initform nil :initarg :pregrasp-pose)
   (arms :accessor arms :initform nil :initarg :arms)
   (close-radius :accessor close-radius :initform nil :initarg :close-radius)
   (safe-pose :accessor safe-pose :initform nil :initarg :safe-pose)
   (effort :accessor effort :initform nil :initarg :effort)
   (grasp-type :accessor grasp-type :initform nil :initarg :grasp-type)))

(defgeneric call-action (action &rest params))

(defmethod call-action ((action-sym t) &rest params)
  (ros-info (pr2 manip-pm)
   "Unimplemented operation `~a' with parameters ~a. Doing nothing."
   action-sym params)
  (sleep 0.5))

(defmethod call-action :around (action-sym &rest params)
  (ros-info (pr2 manip-pm)
                    "Executing manipulation action ~a ~a."
                    action-sym params)
  (prog1 (call-next-method)
    (ros-info (pr2 manip-pm) "Manipulation action done.")))

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(defmacro lazy-try-until (variable-name lazy-slot-name lazy-values &body body)
  `(block try-block
     (flet ((success ()
              (return-from try-block t)))
       (loop while (lazy-car ,lazy-values)
             do (let ((,variable-name (var-value ',lazy-slot-name
                                                 (lazy-car ,lazy-values))))
                  ,@body)
                (setf ,lazy-values (lazy-cdr ,lazy-values))))))

(def-action-handler park (arms obj &optional obstacles)
  (declare (ignore obstacles))
  (let ((arms (force-ll arms)))
    (ros-info (pr2 park) "Park arms ~a" arms)
    (when (> (length arms) 1)
      (let ((trajectories
              (mapcar (lambda (arm)
                        (let* ((frame-id
                                 (ecase arm
                                   (:left "l_wrist_roll_link")
                                   (:right "r_wrist_roll_link")))
                               (arm-in-tll
                                 (cl-tf2:ensure-pose-stamped-transformed
                                  *tf2*
                                  (tf:make-pose-stamped
                                   frame-id (ros-time)
                                   (tf:make-identity-vector)
                                   (tf:make-identity-rotation))
                                  "/torso_lift_link" :use-current-ros-time t))
                               (raised
                                 (tf:copy-pose-stamped
                                  arm-in-tll
                                  :origin
                                  (tf:v+
                                   (tf:origin arm-in-tll)
                                   (tf:make-3d-vector -0.1 0 0.1)))))
                          (execute-move-arm-pose
                           arm raised :plan-only t
                                      :allowed-collision-objects
                                      `(,(desig-prop-value obj 'desig-props::name)))))
                      arms)))
        (moveit::execute-trajectories trajectories)))
    (unless (> (length arms) 1)
      (cond
        (obj
         (let ((grasp-type (desig-prop-value obj 'desig-props:grasp-type)))
           (cond
             ((and obj arms)
              (let* ((newest-effective (newest-effective-designator obj))
                     (object-name (desig-prop-value newest-effective
                                                    'desig-props:name))
                     (allowed-collision-objects
                       (append
                        (cond (object-name (list object-name))
                              (t nil))
                        (list "all"))))
                (dolist (arm (force-ll arms))
                  (when arm
                    (let ((ignore-collisions nil))
                      (cpl:with-failure-handling
                          ((cram-plan-failures:manipulation-pose-unreachable (f)
                             (declare (ignore f))
                             (roslisp:ros-warn
                              (pr2 manip-pm)
                              "Park failed. Retrying with collisions ignored.")
                             (setf ignore-collisions t)
                             (cpl:retry))
                           (cram-plan-failures:manipulation-failed (f)
                             (declare (ignore f))
                             (roslisp:ros-warn
                              (pr2 manip-pm)
                              "Park failed. Retrying with collisions ignored.")
                             (setf ignore-collisions t)
                             (cpl:retry))
                           (moveit:planning-failed (f)
                             (declare (ignore f))
                             (cpl:fail
                              'cram-plan-failures:manipulation-pose-unreachable)))
                        (let ((carry-pose
                                (ecase arm
                                  (:left (cond
                                           ((eql grasp-type
                                                 'desig-props:top-slide-down)
                                            (tf:make-pose-stamped
                                             "base_link" (ros-time)
                                             (tf:make-3d-vector 0.3 0.5 1.3)
                                             (tf:euler->quaternion
                                              :ax 0 :ay (/ pi -2))))
                                           (t (tf:make-pose-stamped
                                               "base_link" (ros-time)
                                               (tf:make-3d-vector 0.3 0.5 1.3)
                                               (tf:euler->quaternion :ax 0)))))
                                  (:right (cond
                                            ((eql grasp-type
                                                  'desig-props:top-slide-down)
                                             (tf:make-pose-stamped
                                              "base_link" (ros-time)
                                              (tf:make-3d-vector 0.3 -0.5 1.3)
                                              (tf:euler->quaternion
                                               :ax 0 :ay (/ pi -2))))
                                            (t (tf:make-pose-stamped
                                                "base_link" (ros-time)
                                                (tf:make-3d-vector 0.3 -0.5 1.3)
                                                (tf:euler->quaternion :ax 0))))))))
                          (execute-move-arm-pose
                           arm carry-pose
                           :allowed-collision-objects allowed-collision-objects
                           :ignore-collisions ignore-collisions)))))))))))
        (t (dolist (arm arms)
             (assume-registered-arm-pose 'park-low arm)))))))

(def-action-handler lift (obj arms distance)
  (let ((arms (force-ll arms))
        (obj-name (desig-prop-value obj 'desig-props:name)))
    (cond ((eql (length arms) 1)
           (lift-grasped-object-with-one-arm (first arms) distance))
          ((eql (length arms) 2)
           (lift-grasped-object-with-two-arms obj-name arms distance))
          (t (error 'simple-error :format-control "No arms for lifting inferred.")))))

(define-hook cram-language::on-begin-grasp (obj-desig))
(define-hook cram-language::on-finish-grasp (log-id success))
(define-hook cram-language::on-grasp-decisions-complete
    (log-id object-name pregrasp-pose grasp-pose side object-pose))

(defun update-action-designator (action-desig new-properties)
  (make-designator 'action (update-designator-properties
                            new-properties
                            (description action-desig))
                   action-desig))

(defun perform-grasp (action-desig object assignments-list &key log-id)
  (let* ((obj (desig:newest-effective-designator object))
         (obj-pose (reference (desig-prop-value obj 'desig-props:at)))
         (obj-name (desig-prop-value obj 'desig-props:name)))
    (labels ((grasp-parameters (assignment)
               (let* ((pose (pose assignment))
                      (side (side assignment))
                      (grasp-type (grasp-type assignment))
                      (gripper-offset (gripper-offset side))
                      (pregrasp-pose
                        (cl-tf2:ensure-pose-stamped-transformed
                         *tf2*
                         (relative-grasp-pose
                          (relative-grasp-pose pose (pregrasp-offset assignment))
                          gripper-offset)
                         "/torso_lift_link"))
                      (grasp-pose
                        (cl-tf2:ensure-pose-stamped-transformed
                         *tf2*
                         (relative-grasp-pose
                          (relative-grasp-pose pose (grasp-offset assignment))
                          gripper-offset)
                         "/torso_lift_link"))
                      (effort 100))
                 (make-instance
                  'grasp-parameters
                  :pregrasp-pose pregrasp-pose
                  :grasp-pose grasp-pose
                  :grasp-type grasp-type
                  :arms `(,side)
                  :close-radius (or (close-radius assignment)
                                    0.0)
                  :safe-pose (ecase side
                               (:left *left-safe-pose*)
                               (:right *right-safe-pose*))
                  :effort effort))))
      (let ((params (mapcar #'grasp-parameters assignments-list)))
        (cpl:with-failure-handling
            ((cram-plan-failures:manipulation-failure (f)
               (declare (ignore f))
               (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable)))
          ;; TODO(winkler): Fix this dirty hack. Here, only the first
          ;; assignment is used. This is wrong.
          (dolist (param-set params)
            (cram-language::on-grasp-decisions-complete
             log-id
             obj-name (pregrasp-pose param-set)
             (grasp-pose param-set)
             (map 'vector #'identity (arms param-set)) obj-pose))
          (execute-grasps obj-name params)
          (update-action-designator
           action-desig
           `((arm ,(first (arms (first params))))
             (effort ,(effort (first params)))
             (grasp-type ,(grasp-type (first params))))))
        (dolist (param-set params)
          (with-vars-strictly-bound (?link-name)
              (lazy-car
               (prolog
                `(cram-manipulation-knowledge:end-effector-link
                  ,(first (arms param-set)) ?link-name)))
            (plan-knowledge:on-event
             (make-instance 'plan-knowledge:object-attached
                            :object obj
                            :link ?link-name
                            :side (first (arms param-set))))))))))

(def-action-handler grasp (action-desig object)
  (let ((grasp-assignments (crs:prolog `(grasp-assignments
                                         ,object ?grasp-assignments)))
        (log-id (first (cram-language::on-begin-grasp object)))
        (success nil))
    (unwind-protect
         (unless (lazy-try-until assignments-list
                     ?grasp-assignments grasp-assignments
                   (block next-assignment-list
                     (cpl:with-failure-handling
                         ((cram-plan-failures:manipulation-pose-unreachable (f)
                            (declare (ignore f))
                            (ros-warn (pr2 manip-pm)
                                      "Try next grasp assignment")
                            (return-from next-assignment-list)))
                       (ros-info (pr2 manip-pm) "Executing grasp(s):~%")
                       (dolist (assignment assignments-list)
                         (ros-info (pr2 manip-pm) " - ~a/~a"
                                   (grasp-type assignment)
                                   (side assignment)))
                       (perform-grasp action-desig object assignments-list
                                      :log-id log-id)
                       (ros-info (pr2 manip-pm) "Succeeded in grasping")
                       (setf success t)
                       (success))))
           (cpl:fail 'manipulation-pose-unreachable))
      (cram-language::on-finish-grasp log-id success))))

(def-action-handler grasp-too-far (object)
  (declare (ignore object))
  (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))

(defun pose-pointing-away-from-base (object-pose)
  (let ((ref-frame "/base_link")
        (fin-frame "/map"))
    (let* ((base-transform-map
             (cl-tf2:ensure-transform-available
              *tf2* ref-frame fin-frame))
           (base-pose-map (tf:make-pose-stamped
                           (tf:frame-id base-transform-map)
                           (tf:stamp base-transform-map)
                           (tf:translation base-transform-map)
                           (tf:rotation base-transform-map)))
           (object-pose-map (tf:transform-pose
                             *tf*
                             :pose object-pose
                             :target-frame fin-frame))
           (origin1 (tf:origin base-pose-map))
           (origin2 (tf:origin object-pose-map))
           (p1 (tf:make-3d-vector (tf:x origin1) (tf:y origin1) 0.0))
           (p2 (tf:make-3d-vector (tf:x origin2) (tf:y origin2) 0.0))
           (angle (+ (* (signum (- (tf:y p2) (tf:y p1)))
                        (acos (/ (- (tf:x p2) (tf:x p1)) (tf:v-dist p1 p2))))
                     (/ pi -2))))
      (tf:make-pose-stamped fin-frame 0.0
                            (tf:origin object-pose-map)
                            (tf:euler->quaternion :az (+ angle (/ pi 2)))))))

(define-hook cram-language::on-begin-putdown (obj-desig loc-desig))
(define-hook cram-language::on-finish-putdown (log-id success))

(defun make-putdown-pose (putdown-location)
  (let* ((putdown-pose (pose-pointing-away-from-base
                        (reference putdown-location)))
         (pose-in-tll
           (cl-tf2:ensure-pose-stamped-transformed
            *tf2* putdown-pose "/torso_lift_link" :use-current-ros-time t)))
    (tf:copy-pose-stamped
     pose-in-tll :origin (tf:v+ (tf:origin pose-in-tll)
                                ;; artificial offset for the putdown pose
                                (tf:make-3d-vector 0.0 0.0 0.05)))))

(defun orient-pose (pose-stamped z-rotation)
  (cond ((eql (coerce z-rotation 'short-float) (coerce 0 'short-float))
         pose-stamped) ;; No rotation, just return the original pose
        (t (let* ((orig-orient (tf:orientation pose-stamped))
                  (tran-orient (tf:orientation
                                (cl-transforms:transform-pose
                                 (tf:make-transform
                                  (tf:make-identity-vector)
                                  (tf:euler->quaternion :az z-rotation))
                                 (tf:make-pose
                                  (tf:make-identity-vector) orig-orient)))))
             (tf:make-pose-stamped
              (tf:frame-id pose-stamped) (ros-time)
              (tf:origin pose-stamped) tran-orient)))))

(define-hook cram-language::on-put-down-reorientation-count (object-designator))

(def-action-handler put-down (object-designator
                              location grasp-assignments grasp-type
                              max-tilt holding-grippers)
  "Delegates the type of the put down action which suppose to be
executed for the currently type of grasped object."
  (unless (and object-designator location)
    (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
  (assert (> (length grasp-assignments) 0) ()
          "No arm/pose pairs specified during put-down.")
  (let* ((log-id (first (cram-language::on-begin-putdown object-designator location)))
         (success nil)
         (putdown-pose-pure (make-putdown-pose location))
         (putdown-orientations (or (first (cram-language::on-put-down-reorientation-count
                                           object-designator))
                                   8)) ;; Try different orientations when placing the object
         (current-orientation 0)
         (pre-putdown-offset (cond ((eql grasp-type 'desig-props:top-slide-down)
                                    *pre-putdown-offset*)
                                   (t *pre-putdown-offset*)))
         (putdown-offset (cond ((eql grasp-type 'desig-props:top-slide-down)
                                *putdown-offset*)
                               (t *putdown-offset*)))
         (unhand-offset (cond ((eql grasp-type 'desig-props:top-slide-down)
                               *unhand-top-slide-down-offset*)
                              (t *unhand-offset*))))
    (unwind-protect
         (progn
           (cpl:with-failure-handling
               ((manipulation-failure (f)
                  (declare (ignore f))
                  (when (< current-orientation putdown-orientations)
                    (incf current-orientation)
                    (cpl:retry))))
             (let* ((orientation-offset (* 2 pi (/ current-orientation
                                                   putdown-orientations)))
                    (putdown-pose (orient-pose putdown-pose-pure
                                               orientation-offset)))
               (publish-pose putdown-pose "/putdownpose")
               (let ((grasp-assignment (first grasp-assignments)))
                 (flet ((target-gripper-pose (object-in-gripper-pose
                                              target-object-pose)
                          (let* ((object-in-gripper
                                   (tf:pose->transform
                                    object-in-gripper-pose))
                                 (gripper-in-object
                                   (tf:transform-inv object-in-gripper))
                                 (object-in-world
                                   (tf:pose->transform
                                    (cl-transforms:transform-pose
                                     (tf:make-transform
                                      ;; Slight offset above the final
                                      ;; pose to ensure collision
                                      ;; detection doesn't spoil
                                      ;; everything (imprecise models
                                      ;; and everything).
                                      (tf:make-3d-vector 0 0 0.025)
                                      (tf:make-identity-rotation))
                                     target-object-pose)))
                                 (gripper-in-world
                                   (tf:transform*
                                    object-in-world gripper-in-object)))
                            (tf:pose->pose-stamped
                             (tf:frame-id target-object-pose)
                             (ros-time)
                             (tf:transform->pose gripper-in-world)))))
                   (flet ((gripper-grasp-pose (pose-offset)
                            (relative-grasp-pose
                             (target-gripper-pose
                              (slot-value grasp-assignment
                                          'pose)
                              putdown-pose)
                             pose-offset)))
                     (let* ((side (slot-value grasp-assignment 'side))
                            (pre-putdown-pose (gripper-grasp-pose
                                               pre-putdown-offset))
                            (putdown-hand-pose (gripper-grasp-pose
                                                putdown-offset))
                            (unhand-pose (gripper-grasp-pose
                                          unhand-offset)))
                       (publish-pose putdown-hand-pose "/putdownhandpose")
                       (cpl:with-failure-handling
                           ((cram-plan-failures:manipulation-failed (f)
                              (declare (ignore f))
                              (cpl:fail
                               'cram-plan-failures:manipulation-pose-unreachable)))
                         (let ((link-name
                                 (cut:var-value
                                  '?link
                                  (first
                                   (crs:prolog
                                    `(manipulator-link ,side ?link)))))
                               (planning-group
                                 (cut:var-value
                                  '?group
                                  (first
                                   (crs:prolog
                                    `(planning-group ,side ?group))))))
                           (cond ((moveit:plan-link-movements
                                   link-name planning-group
                                   `(,pre-putdown-pose
                                     ,putdown-hand-pose
                                     ,unhand-pose)
                                   :destination-validity-only t
                                   :allowed-collision-objects
                                   (list (desig-prop-value
                                          object-designator
                                          'desig-props:name))
                                   :max-tilt max-tilt)
                                  (execute-putdown
                                   :side side
                                   :object-name (desig-prop-value
                                                 object-designator
                                                 'desig-props:name)
                                   :pre-putdown-pose pre-putdown-pose
                                   :putdown-pose putdown-hand-pose
                                   :unhand-pose unhand-pose
                                   :safe-pose (ecase side
                                                (:left *left-safe-pose*)
                                                (:right *right-safe-pose*))
                                   :max-tilt max-tilt)
                                  (setf success t))
                                 (t
                                  (cpl:fail
                                   'manipulation-pose-unreachable)))))))))))
           (loop for grasp-assignment in grasp-assignments
                 for side = (slot-value grasp-assignment 'side)
                 for grasped-object = (or (car (slot-value grasp-assignment
                                                           'handle-pair))
                                          object-designator)
                 do (with-vars-strictly-bound (?link-name)
                        (lazy-car
                         (prolog
                          `(cram-manipulation-knowledge:end-effector-link
                            ,side ?link-name)))
                      (plan-knowledge:on-event
                       (make-instance
                        'plan-knowledge:object-detached
                        :object object-designator
                        :link ?link-name
                        :side side)))))
      (cram-language::on-finish-putdown log-id success))))
