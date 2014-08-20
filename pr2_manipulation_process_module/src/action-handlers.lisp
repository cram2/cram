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
  (ros-info (pr2 park) "Park arms ~a ~a" obj arms)
  (let ((grasp-type (desig-prop-value obj 'desig-props:grasp-type)))
    (cond ((and obj arms)
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
                               (:left (cond ((eql grasp-type 'desig-props:top-slide-down)
                                             (tf:make-pose-stamped
                                              "base_link" (ros-time)
                                              (tf:make-3d-vector 0.3 0.5 1.3)
                                              (tf:euler->quaternion :ax 0 :ay (/ pi -2))))
                                            (t (tf:make-pose-stamped
                                                "base_link" (ros-time)
                                                (tf:make-3d-vector 0.3 0.5 1.3)
                                                (tf:euler->quaternion :ax 0)))))
                               (:right (cond ((eql grasp-type 'desig-props:top-slide-down)
                                              (tf:make-pose-stamped
                                               "base_link" (ros-time)
                                               (tf:make-3d-vector 0.3 -0.5 1.3)
                                               (tf:euler->quaternion :ax 0 :ay (/ pi -2))))
                                             (t (tf:make-pose-stamped
                                                 "base_link" (ros-time)
                                                 (tf:make-3d-vector 0.3 -0.5 1.3)
                                                 (tf:euler->quaternion :ax 0))))))))
                       (execute-move-arm-pose
                        arm carry-pose
                        :allowed-collision-objects allowed-collision-objects
                        :ignore-collisions ignore-collisions)))))))))))

(def-action-handler lift (arms distance)
  (force-ll arms)
  (cond ((eql (length arms) 1)
         (lift-grasped-object-with-one-arm (first arms) distance))
        (t (error 'simple-error :format-control "No arms for lifting inferred."))))

(define-hook on-begin-grasp (obj-desig))
(define-hook on-finish-grasp (log-id success))
(define-hook on-grasp-decisions-complete
    (object-name pregrasp-pose grasp-pose side object-pose))

(defun perform-grasp (object grasp-assignment pregrasp-offset grasp-offset)
  (let* ((obj (desig:newest-effective-designator object))
         (obj-pose (reference (desig-prop-value obj 'desig-props:at)))
         (pose (slot-value grasp-assignment 'pose))
         (side (slot-value grasp-assignment 'side))
         (pregrasp-pose-tll
           (moveit:ensure-pose-stamped-transformed
            (relative-grasp-pose pose pregrasp-offset)
            "/torso_lift_link"))
         (grasp-pose-tll
           (moveit:ensure-pose-stamped-transformed
            (relative-grasp-pose pose grasp-offset)
            "/torso_lift_link"))
         (close-radius (or (slot-value grasp-assignment
                                       'close-radius)
                           0.0))
         (grasped-object (or (slot-value grasp-assignment
                                         'handle-pair)
                             obj)))
    (declare (ignore grasped-object))
         (cpl:with-failure-handling
             ((cram-plan-failures:manipulation-failure (f)
                (declare (ignore f))
                (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable)))
           (publish-pose pregrasp-pose-tll "/dhdhdh")
           (prog2
               (on-grasp-decisions-complete
                (desig-prop-value obj 'desig-props:name)
                pregrasp-pose-tll grasp-pose-tll side obj-pose)
               (execute-grasp
                :object-name (desig-prop-value obj 'desig-props:name)
                :object-pose obj-pose
                :pregrasp-pose pregrasp-pose-tll
                :grasp-pose grasp-pose-tll
                :side side
                :gripper-close-pos close-radius
                :safe-pose (ecase side
                             (:left *left-safe-pose*)
                             (:right *right-safe-pose*)))))
    (with-vars-strictly-bound (?link-name)
        (lazy-car
         (prolog
          `(cram-manipulation-knowledge:end-effector-link
            ,side ?link-name)))
      (plan-knowledge:on-event
       (make-instance 'plan-knowledge:object-attached
                      :object obj
                      :link ?link-name
                      :side side)))))

(defun perform-grasps (object lazy-grasp-assignment pregrasp-offset grasp-offset)
  (let ((log-id (first (on-begin-grasp object)))
        (success nil))
    (unwind-protect
         (unless (lazy-try-until
                     grasp-assignment ?grasp-assignment lazy-grasp-assignment
                   (block next-grasp-assignment
                     (cpl:with-failure-handling
                         ((cram-plan-failures:manipulation-pose-unreachable (f)
                            (declare (ignore f))
                            (return-from next-grasp-assignment)))
                       (perform-grasp
                        object grasp-assignment pregrasp-offset grasp-offset)
                       (setf success t)
                       (success))))
           (cpl:fail 'manipulation-pose-unreachable))
      (on-finish-grasp log-id success))))
      
(defun perform-lazy-grasps (obj avail-arms pregrasp-offset grasp-offset
                            &key reorient-object)
  (let* ((ga-prolog (crs:prolog `(grasp-handle-assignment ;; optimal-handle-grasp
                                  ,obj
                                  ,avail-arms
                                  ,pregrasp-offset
                                  ,grasp-offset
                                  ,reorient-object
                                  ?grasp-assignment))))
    (when (lazy-car ga-prolog)
      (ros-info (pr2 manip-pm) "Found grasp assignment(s)."))
    (perform-grasps obj ga-prolog pregrasp-offset grasp-offset)))

(def-action-handler grasp (obj-desig available-arms)
  (let ((grasp-types (crs:prolog `(grasp-type ,obj-desig ?grasp-type))))
    (lazy-try-until grasp-type ?grasp-type grasp-types
      (ros-info (pr2 manip-pm) "Executing grasp-type: ~a" grasp-type)
      (cond ((eql grasp-type 'desig-props:top-slide-down)
             (perform-lazy-grasps obj-desig available-arms
                                  *pregrasp-top-slide-down-offset*
                                  *grasp-offset*))
            ((eql grasp-type 'desig-props:push)
             (perform-lazy-grasps obj-desig available-arms
                                  *pregrasp-offset*
                                  *grasp-offset*
                                  :reorient-object t)))
      (success))))

(defun pose-pointing-away-from-base (object-pose)
  (let ((ref-frame "/base_link")
        (fin-frame "/map"))
    (let* ((base-transform-map
             (moveit:ensure-transform-available
              ref-frame fin-frame))
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

(define-hook on-begin-putdown (obj-desig loc-desig))
(define-hook on-finish-putdown (log-id success))

(defun make-putdown-pose (putdown-location)
  (let* ((putdown-pose (pose-pointing-away-from-base
                        (reference putdown-location)))
         (pose-in-tll
           (moveit:ensure-pose-stamped-transformed
            putdown-pose "/torso_lift_link" :ros-time t)))
    (tf:copy-pose-stamped
     pose-in-tll :origin (tf:v+ (tf:origin pose-in-tll)
                                ;; artificial offset for the putdown pose
                                (tf:make-3d-vector 0.0 0.0 0.02)))))

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

(define-hook on-put-down-reorientation-count (object-designator))

(def-action-handler put-down (object-designator location grasp-assignments grasp-type)
  "Delegates the type of the put down action which suppose to be executed
for the currently type of grasped object."
  (unless (and object-designator location)
    (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
  (assert (> (length grasp-assignments) 0) ()
          "No arm/pose pairs specified during put-down.")
  (let* ((log-id (first (on-begin-putdown object-designator location)))
         (success nil)
         (putdown-pose-pure (make-putdown-pose location))
         (putdown-orientations (or (first (on-put-down-reorientation-count object-designator))
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
                                      (tf:make-3d-vector
                                       0 0 (or ;(abs (desig-prop-value
                                               ;      object-designator
                                               ;      'desig-props:z-offset))
                                               0.0))
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
                                          'desig-props:name)))
                                  (execute-putdown
                                   :side side
                                   :object-name (desig-prop-value
                                                 object-designator
                                                 'desig-props:name)
                                   :pre-putdown-pose pre-putdown-pose
                                   :putdown-pose putdown-hand-pose
                                   :unhand-pose unhand-pose)
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
      (on-finish-putdown log-id success))))
