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
          do (roslisp:publish
              (roslisp:advertise
               "/dbg" "geometry_msgs/PoseStamped")
              (tf:pose-stamped->msg pose-grasp))
             (execute-move-arm-pose side pose-grasp))))

(def-action-handler debug ()
  (roslisp:ros-info () "Debug handler called.~%"))

(def-action-handler container-closed (handle side)
  (let* ((handle-pose (designator-pose (newest-effective-designator handle)))
         (new-object-pose (close-drawer handle-pose side)))
    (update-object-designator-pose handle new-object-pose)))

(def-action-handler park (obj arms &optional obstacles)
  (roslisp:ros-info (pr2-manipulation-process-module) "Park arms ~a ~a"
                    obj arms)
  (force-ll arms)
  (cond ((and (not arms) (not obj))
         (cpl:par-loop (arm arms)
           (when arm
             (let ((carry-pose (ecase arm
                                 (:right *carry-pose-right*)
                                 (:left *carry-pose-left*))))
               (execute-arm-trajectory
                arm (ik->trajectory (get-ik arm carry-pose)))))))
        ((and arms (not obj))
         (when obstacles
           (clear-collision-objects)
           (sem-map-coll-env:publish-semantic-map-collision-objects)
           (dolist (obstacle (cut:force-ll obstacles))
             (register-collision-object obstacle)))
         (cpl:par-loop (arm arms)
           (when arm
             (let ((orientation (calculate-carry-orientation
                                 obj arm
                                 (list *top-grasp*
                                       (cl-transforms:make-identity-rotation))))
                   (carry-pose (ecase arm
                                 (:right *carry-pose-right*)
                                 (:left *carry-pose-left*))))
               (if orientation
                   (execute-move-arm-pose
                    arm
                    (tf:copy-pose-stamped carry-pose :orientation orientation)
                    :allowed-collision-objects (list "\"all\""))
                   (execute-move-arm-pose
                    arm carry-pose
                    :allowed-collision-objects (list "\"all\"")))))))
        ((eql (length arms) 1)
         (park-grasped-object-with-one-arm obj (first arms) obstacles))
        ((eql (length arms) 2)
         (park-grasped-object-with-two-arms obj arms obstacles))
        ((> (length arms) 1)
         (error 'simple-error :format-control "Parking with several arms not implemented, yet."))
        (t (error 'simple-error :format-control "No arms for parking inferred."))))

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

(defgeneric hook-before-grasp (obj-desig))
(defmethod hook-before-grasp (obj-desig))

(defgeneric hook-after-grasp (log-id success))
(defmethod hook-after-grasp (log-id success))

(def-action-handler grasp (obj-desig grasp-assignments obstacles)
  (declare (ignore obstacles))
  (roslisp:ros-info () "Beginning grasp action handler")
  (let ((pair-count (length grasp-assignments)))
    (assert (> pair-count 0) ()
            "No arm/pose pairs specified for grasping.")
    (cpl:par-loop (grasp-assignment grasp-assignments)
      (let* ((obj-pose (reference (desig-prop-value obj-desig
                                                    'desig-props:at)))
             (pose (slot-value grasp-assignment 'pose))
             (time (roslisp:ros-time)))
        (cond ((tf:wait-for-transform
                *tf*
                :source-frame (tf:frame-id pose)
                :target-frame "/torso_lift_link"
                :timeout 5.0
                :time time)
               (let* ((side (slot-value grasp-assignment 'side))
                      (pregrasp-pose (relative-grasp-pose
                                      pose *pregrasp-offset*))
                      (pregrasp-pose-tll (tf:transform-pose
                                          *tf*
                                          :pose pregrasp-pose
                                          :target-frame "/torso_lift_link"))
                      (grasp-pose (relative-grasp-pose
                                   pose *grasp-offset*))
                      (grasp-pose-tll (tf:transform-pose
                                       *tf*
                                       :pose grasp-pose
                                       :target-frame "/torso_lift_link"))
                      (close-radius (or (slot-value grasp-assignment
                                                    'close-radius)
                                        0.0)))
                 (let ((log-id (hook-before-grasp obj-desig)))
                   (cpl:with-failure-handling
                       ((cram-plan-failures:manipulation-failed (f)
                          (declare (ignore f))
                          (hook-after-grasp log-id nil)
                          (cpl:fail
                           'cram-plan-failures:manipulation-pose-unreachable)))
                     (prog1
                         (execute-grasp :object-name (desig-prop-value
                                                      obj-desig
                                                      'desig-props:name)
                                        :object-pose obj-pose
                                        :pregrasp-pose pregrasp-pose-tll
                                        :grasp-pose grasp-pose-tll
                                        :side side
                                        :gripper-close-pos close-radius)
                       (hook-after-grasp log-id t))))))
              (t (cpl:error 'manipulation-failed)))))
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
    (tf:wait-for-transform *tf* :source-frame ref-frame
                                :target-frame fin-frame
                                :time (roslisp:ros-time))
    (tf:wait-for-transform
     *tf*
     :source-frame (tf:frame-id object-pose)
     :target-frame fin-frame
     :time (roslisp:ros-time))
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

(defgeneric hook-before-putdown (obj-desig loc-desig))
(defmethod hook-before-putdown (obj-desig loc-desig))

(defgeneric hook-after-putdown (log-id success))
(defmethod hook-after-putdown (log-id success))

(def-action-handler put-down (object-designator
                              location grasp-assignments obstacles)
  (declare (ignore obstacles))
  "Delegates the type of the put down action which suppose to be executed
for the currently type of grasped object."
  (let ((time (roslisp:ros-time))
        (pair-count (length grasp-assignments)))
    (assert (> pair-count 0) ()
            "No arm/pose pairs specified during put-down.")
    ;; NOTE(winkler): The actual reasoning concerning object-in-hand
    ;; information and used grippers per grasped object part has been
    ;; done in the designator.lisp prolog patterns already. Here, we
    ;; just execute whatever result the reasoning yielded.
    (let* ((putdown-pose (pose-pointing-away-from-base
                          (reference location))))
      (cond ((tf:wait-for-transform
              *tf*
              :timeout 5.0
              :time time
              :source-frame (tf:frame-id putdown-pose)
              :target-frame "/torso_lift_link")
             (let ((new-obj-pose
                     (tf:pose->pose-stamped
                      "/torso_lift_link"
                      time
                      (cl-transforms:transform-pose
                       (tf:make-transform
                        (tf:make-3d-vector 0.0 0.0 0.0)
                        (tf:make-identity-rotation))
                       (tf:transform-pose
                        *tf* :pose (tf:copy-pose-stamped
                                    putdown-pose
                                    :stamp time)
                             :target-frame "/torso_lift_link"))))
                   (arm-poses
                     (loop for grasp in grasp-assignments
                           for arm = (slot-value grasp 'side)
                           for pose = (slot-value grasp 'pose)
                           collect (cons arm pose))))
               (cpl:par-loop (arm-pose arm-poses)
                 (let* ((pose-in-gripper (cdr arm-pose))
                        (side (car arm-pose))
                        (trafo-into-gripper
                          (tf:transform-inv
                           (tf:pose->transform pose-in-gripper))))
                   (flet ((apply-gripper-trafo (pose-stamped)
                            (tf:pose->pose-stamped
                             (tf:frame-id pose-stamped)
                             (roslisp:ros-time)
                             (tf:transform
                              (tf:make-transform
                               (tf:translation trafo-into-gripper)
                               (tf:make-identity-rotation))
                              (tf:transform
                               (tf:pose->transform pose-stamped)
                               (tf:make-pose
                                (tf:make-identity-vector)
                                (tf:rotation trafo-into-gripper)))))))
                     (let ((pre-putdown-pose
                             (apply-gripper-trafo
                              (relative-grasp-pose
                               new-obj-pose *pre-putdown-offset*)))
                           (putdown-pose
                             (apply-gripper-trafo
                              (relative-grasp-pose
                               new-obj-pose *putdown-offset*)))
                           (unhand-pose
                             (apply-gripper-trafo
                              (relative-grasp-pose
                               new-obj-pose *unhand-offset*))))
                       (roslisp:publish
                        (roslisp:advertise
                         "/dbg" "geometry_msgs/PoseStamped")
                        (tf:pose-stamped->msg pre-putdown-pose))
                       (let ((log-id (hook-before-putdown object-designator
                                                          location)))
                         (cpl:with-failure-handling
                             ((cram-plan-failures:manipulation-failed (f)
                                (declare (ignore f))
                                (hook-after-putdown log-id nil)
                                (cpl:fail
                                 'cram-plan-failures:manipulation-pose-unreachable)))
                           (execute-putdown
                            :side side
                            :object-name (desig-prop-value object-designator
                                                           'desig-props:name)
                            :pre-putdown-pose pre-putdown-pose
                            :putdown-pose putdown-pose
                            :unhand-pose unhand-pose)
                           (hook-after-putdown log-id t))))))))
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
                          'plan-knowledge:object-attached
                          :object object-designator
                          :link ?link-name
                          :side side)))))
            (t (cpl:error 'manipulation-failed))))))

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
                            (roslisp:ros-time)
                            arm-pose)))
    (roslisp:publish (roslisp:advertise "/objputdownpose"
                                        "geometry_msgs/PoseStamped")
                     (tf:pose-stamped->msg obj-pose))
    (roslisp:publish (roslisp:advertise "/objputdownposearm"
                                        "geometry_msgs/PoseStamped")
                     (tf:pose-stamped->msg arm-pose-stamped))
    arm-pose-stamped))
     
(def-action-handler constraint-motion-handler (motion-phases world-object-desig tool-object-desigs)
  ;; first assemble all the object designators lists
  (let* ((tool-desigs (force-ll tool-object-desigs))
         (all-object-desigs (cons world-object-desig tool-desigs)))
  ;; then publish the transforms of the object designators on tf
  (with-tf-publishing (all-object-desigs)
    ;; then assemble all the constraint descriptions from knowrob
    (let* ((object-name (extract-knowrob-name world-object-desig))
           (tool-names (mapcar #'extract-knowrob-name tool-desigs))
           (constraint-phases (query-knowrob-constraints-action
                               motion-phases object-name tool-names)))
      ;; now execute the motion-phases, one by one
      ;; note: we have to descriminate between the arms
      (mapcan #'(lambda (phase)
                  (flet ((execute-arm-constraints (arm-constraints tools)
                           ;; a function to execute a list containing the cons of
                           ;; one 'side' and a list of constraints as returned from the knowrob queries
                           (let ((side (arm-from-knowrob-name
                                        (first arm-constraints)
                                        tools))
                                 (constraints (rest arm-constraints)))
                             (execute-constraints-motion constraints side))))
                    (cond
                      ((= (length phase) 1)
                       (execute-arm-constraints (first phase) tool-desigs))
                      ((= (length phase) 2)
                       (cpl-impl:par 
                         (execute-arm-constraints (first phase) tool-desigs)
                         (execute-arm-constraints (second phase) tool-desigs)))
                      (t (error 'simple-error 
                              :format-control "Only one or two sets of constraints valid for one motion phase. Given list had '%a' elements.~%"
                              :format-arguments (list (length phase)))))))
              constraint-phases)
      constraint-phases))))
