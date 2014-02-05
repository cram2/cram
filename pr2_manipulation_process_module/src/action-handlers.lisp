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

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(def-action-handler container-opened (object grasp-assignments opening-measure)
  (call-action 'grasp object grasp-assignments nil)
  (let* ((grasp-assignment (first grasp-assignments))
         (handle-pair (slot-value grasp-assignment 'handle-pair))
         (handle-relative (car handle-pair))
         (handle-absolute (cdr handle-pair))
         (side (slot-value grasp-assignment 'side))
         (opening-step 5.0)
         (steps (loop for i from opening-step to opening-measure
                        by opening-step
                      collect (open-angle object handle-relative (- i)))))
    (declare (ignore handle-absolute))
    (loop for step in steps
          ;; TODO(winkler): Refactor this to send a series of poses in
          ;; ONE movement description to MoveIt! instead. This way,
          ;; the motion planner can also tell us whether the opening
          ;; will work or not. Also: Do this _BEFORE_ grasping the
          ;; handle.
          for pose-grasp = (relative-grasp-pose
                            step *grasp-offset*)
          do (publish-pose pose-grasp "/dbg")
             (execute-move-arm-pose side pose-grasp))))

(def-action-handler debug ()
  (ros-info (pr2 debug) "Debug handler called.~%"))

(def-action-handler container-closed (handle side)
  (let* ((handle-pose (designator-pose (newest-effective-designator handle)))
         (new-object-pose (close-drawer handle-pose side)))
    (update-object-designator-pose handle new-object-pose)))

(def-action-handler park (obj arms &optional obstacles)
  (declare (ignore obstacles))
  (ros-info (pr2 park) "Park arms ~a ~a" obj arms)
  (cond ((and obj arms)
         (let* ((newest-effective (newest-effective-designator obj))
                (object-name (desig-prop-value newest-effective
                                               'desig-props:name))
                (allowed-collision-objects
                  (append
                   (cond (object-name (list object-name))
                         (t nil))
                   (list "all"))))
           (cpl:par-loop (arm (force-ll arms))
             (when arm
               (let ((ignore-collisions nil))
                 (cpl:with-failure-handling
                     ((cram-plan-failures:manipulation-pose-unreachable (f)
                        (declare (ignore f))
                        (setf ignore-collisions t)
                        (cpl:retry)))
                   (let ((carry-pose
                           (ecase arm
                             (:left (tf:make-pose-stamped
                                     "base_link" (ros-time)
                                     (tf:make-3d-vector 0.3 0.5 1.3)
                                     (tf:euler->quaternion :ax 0))) ;; was pi
                             (:right (tf:make-pose-stamped
                                      "base_link" (ros-time)
                                      (tf:make-3d-vector 0.3 -0.5 1.3)
                                      (tf:euler->quaternion :ax 0)))))) ;; was pi
                     (execute-move-arm-pose
                      arm carry-pose
                      :allowed-collision-objects allowed-collision-objects
                      :ignore-collisions ignore-collisions))))))))))

(def-action-handler lift (arms distance)
  ;; Note(Georg) Curious! We do not need the object designator
  ;; for execution of this action?
  ;; NOTE(winkler): No, we actually don't. The lifting is done by just
  ;; recalculating the new position based on the old one and the fact
  ;; that the gripper should stick to it's current orientation.
  (force-ll arms)
  (cond ((eql (length arms) 1)
         (lift-grasped-object-with-one-arm (first arms) distance))
        ;; TODO(Georg): the next cases is actually deprecated because
        ;; it still relies on the :both arms setup
        
        ;; NOTE(winkler): Apparently it is replaced by a `(par'
        ;; solution already. Nevertheless, it should be extended to be
        ;; more general and support `n' arms when lifting (although we
        ;; only have :left and :right on the PR2). This current
        ;; solution works great, though.
        ((> (length arms) 1)
         (lift-grasped-object-with-both-arms distance))
        (t (error 'simple-error :format-control "No arms for lifting inferred."))))

(def-action-handler pull (obj-desig arms direction distance obstacles)
  (declare (ignore obj-desig obstacles))
  (cpl:par-loop (arm (force-ll arms))
    (let* ((rel-ik (relative-linear-translation->ik
                    arm
                    :x (* (tf:x direction) distance)
                    :y (* (tf:y direction) distance)
                    :z (* (tf:z direction) distance))))
      (when rel-ik
        (execute-arm-trajectory arm (ik->trajectory rel-ik))))))

(def-action-handler push (obj-desig arms direction distance obstacles)
  (declare (ignore obj-desig obstacles))
  (cpl:par-loop (arm (force-ll arms))
    (let* ((rel-ik (relative-linear-translation->ik
                   arm
                   :x (* (tf:x direction) distance)
                   :y (* (tf:y direction) distance)
                   :z (* (tf:z direction) distance))))
      (when rel-ik
        (execute-arm-trajectory arm (ik->trajectory rel-ik))))))

(define-hook on-begin-grasp (obj-desig))
(define-hook on-finish-grasp (log-id success))

(def-action-handler grasp (obj-desig grasp-assignments obstacles)
  (declare (ignore obstacles))
  (ros-info (pr2 grasp) "Beginning grasp action handler")
  (let ((pair-count (length grasp-assignments)))
    (assert (> pair-count 0) ()
            "No arm/pose pairs specified for grasping.")
    (cpl:par-loop (grasp-assignment grasp-assignments)
      (let* ((obj-pose (reference (desig-prop-value
                                   obj-desig 'desig-props:at)))
             (pose-genuine (slot-value grasp-assignment 'pose))
             (pose (tf:make-pose-stamped
                    (tf:frame-id pose-genuine)
                    (tf:stamp pose-genuine)
                    (tf:v+ (tf:origin pose-genuine)
                           (tf:make-3d-vector 0.0 0.0 0.0))
                    (tf:orientation pose-genuine))))
        (publish-pose pose "/dhdhdh")
        (let* ((side (slot-value grasp-assignment 'side))
               (pregrasp-pose-tll
                 (moveit:ensure-pose-stamped-transformed
                  (relative-grasp-pose pose *pregrasp-offset*)
                  "/torso_lift_link"))
               (grasp-pose-tll
                 (moveit:ensure-pose-stamped-transformed
                  (relative-grasp-pose pose *grasp-offset*)
                  "/torso_lift_link"))
               (close-radius (or (slot-value grasp-assignment
                                             'close-radius)
                                 0.0)))
          (let ((log-id (first (on-begin-grasp obj-desig))))
            (cpl:with-failure-handling
                ((cram-plan-failures:manipulation-failed (f)
                   (declare (ignore f))
                   (unwind-protect
                        (cpl:fail
                         'cram-plan-failures:manipulation-pose-unreachable)
                     (on-finish-grasp log-id nil)))
                 (cram-plan-failures:manipulation-pose-unreachable (f)
                   (declare (ignore f))
                   (unwind-protect
                        (cpl:fail
                         'cram-plan-failures:manipulation-pose-unreachable)
                     (on-finish-grasp log-id nil))))
              (prog1
                  (execute-grasp :object-name (desig-prop-value
                                               obj-desig
                                               'desig-props:name)
                                 :object-pose obj-pose
                                 :pregrasp-pose pregrasp-pose-tll
                                 :grasp-pose grasp-pose-tll
                                 :side side
                                 :gripper-close-pos close-radius
                                 :safe-pose (ecase side
                                              (:left *left-safe-pose*)
                                              (:right *right-safe-pose*)))
                (on-finish-grasp log-id t)))))))
    (loop for grasp-assignment in grasp-assignments
          for side = (slot-value grasp-assignment 'side)
          for grasped-object = (or (car (slot-value grasp-assignment
                                                    'handle-pair))
                                   obj-desig)
          do (with-vars-strictly-bound (?link-name)
                 (lazy-car
                  (prolog
                   `(cram-manipulation-knowledge:end-effector-link
                     ,side ?link-name)))
               (plan-knowledge:on-event
                (make-instance
                 'plan-knowledge:object-attached
                 :object obj-desig
                 :link ?link-name
                 :side side))))))

(defun pose-pointing-away-from-base (object-pose)
  (let ((ref-frame "/base_link")
        (fin-frame "/map"))
    (unless
        (and (tf:wait-for-transform
              *tf* :timeout 5.0
                   :source-frame ref-frame
                   :target-frame fin-frame
                   :time (ros-time))
             (tf:wait-for-transform
              *tf* :source-frame (tf:frame-id object-pose)
                   :target-frame fin-frame
                   :time (ros-time)))
      (cpl:fail 'manipulation-pose-unreachable))
    (let* ((base-transform-map (tf:lookup-transform
                                *tf*
                                :time 0.0
                                :source-frame ref-frame
                                :target-frame fin-frame))
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
  (let ((putdown-pose (pose-pointing-away-from-base
                       (reference putdown-location)))
        (time (ros-time)))
    (unless (tf:wait-for-transform
             *tf* :timeout 5.0
                  :time time
                  :source-frame (tf:frame-id putdown-pose)
                  :target-frame "/torso_lift_link")
      (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
    (tf:pose->pose-stamped
     "/torso_lift_link"
     time
     (cl-transforms:transform-pose
      (tf:make-transform
       (tf:make-3d-vector 0.0 0.0 0.02) ;; artificial offset for the putdown pose
       (tf:make-identity-rotation))
      (tf:transform-pose
       *tf* :pose (tf:copy-pose-stamped
                   putdown-pose
                   :stamp time)
            :target-frame "/torso_lift_link")))))

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

(def-action-handler put-down (object-designator
                              location grasp-assignments obstacles)
  (declare (ignore obstacles))
  "Delegates the type of the put down action which suppose to be executed
for the currently type of grasped object."
  (assert (> (length grasp-assignments) 0) ()
          "No arm/pose pairs specified during put-down.")
  (let* ((log-id (first (on-begin-putdown object-designator location)))
         (putdown-pose-pure (make-putdown-pose location))
         (putdown-orientations 1)) ;; Try different orientations
    ;; when placing the object
    (unwind-protect
         (loop for orientation-index from 0 below putdown-orientations
               as orientation-offset = (* 2 pi (/ orientation-index
                                                  putdown-orientations))
               as putdown-pose = (orient-pose putdown-pose-pure
                                              orientation-offset)
               do (publish-pose putdown-pose "/putdownpose")
                  (cpl:par-loop (grasp-assignment grasp-assignments)
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
                                          0 0 (or (desig-prop-value
                                                   object-designator
                                                   'desig-props:z-offset)
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
                                                  *pre-putdown-offset*))
                               (putdown-hand-pose (gripper-grasp-pose
                                                   *putdown-offset*))
                               (unhand-pose (gripper-grasp-pose
                                             *unhand-offset*)))
                          (publish-pose putdown-hand-pose "/putdownhandpose")
                          (cpl:with-failure-handling
                              ((cram-plan-failures:manipulation-failed (f)
                                 (declare (ignore f))
                                 (cpl:fail
                                  'cram-plan-failures:manipulation-pose-unreachable)))
                            (let ((link-name (ecase side
                                               (:left "l_wrist_roll_link")
                                               (:right "r_wrist_roll_link")))
                                  (planning-group (ecase side
                                                    (:left "left_arm")
                                                    (:right "right_arm"))))
                              (cond ((moveit:plan-link-movements
                                      link-name planning-group
                                      `(,pre-putdown-pose
                                        ,putdown-hand-pose
                                        ,unhand-pose)
                                      :destination-validity-only t
                                      :ignore-collisions t
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
                                      :unhand-pose unhand-pose))
                                    (t (cpl:fail
                                        'manipulation-pose-unreachable))))))))))
      (on-finish-putdown log-id nil))
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
                 :side side))))))

(defun calculate-putdown-arm-pose (obj-pose rel-arm-pose)
  (let* ((inv-rel-trafo (tf:make-transform
                         (tf:v- (tf:make-identity-vector)
                                (tf:origin rel-arm-pose))
                         (tf:q- (tf:make-identity-rotation)
                                (tf:orientation rel-arm-pose))))
         (arm-pose (cl-transforms:transform-pose
                    (tf:make-transform (tf:origin obj-pose)
                                       (tf:make-identity-rotation))
                    (cl-transforms:transform-pose
                     (tf:make-transform (tf:make-identity-vector)
                                        (tf:orientation obj-pose))
                     (tf:transform->pose inv-rel-trafo))))
         (arm-pose-stamped (tf:pose->pose-stamped
                            (tf:frame-id obj-pose)
                            (ros-time)
                            arm-pose)))
    (publish-pose obj-pose "/objputdownpose")
    (publish-pose arm-pose-stamped "/objputdownposearm")
    arm-pose-stamped))
