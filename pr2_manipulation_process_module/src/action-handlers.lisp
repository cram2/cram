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

(def-action-handler container-opened (handle side)
  (let* ((handle-pose (designator-pose (newest-effective-designator handle)))
         (new-object-pose (open-drawer handle-pose side)))
    (update-object-designator-pose handle new-object-pose)))

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

(def-action-handler grasp (obj-desig grasp-assignments obstacles)
  (declare (ignore obstacles))
  (roslisp:call-service "/collider_node/reset" "std_srvs/Empty")
  (let ((pair-count (length grasp-assignments)))
    (assert (> pair-count 0) ()
            "No arm/pose pairs specified during grasping.")
    (cpl:par-loop (grasp-assignment grasp-assignments)
      (let ((pose (slot-value grasp-assignment 'pose)))
        (tf:wait-for-transform
         *tf*
         :source-frame (tf:frame-id pose)
         :target-frame "/torso_lift_link"
         :time (roslisp:ros-time))
        (let* ((side (slot-value grasp-assignment 'side))
               (pregrasp-pose (relative-grasp-pose
                               pose *pregrasp-offset*))
               (pregrasp-pose-tll
                 (tf:transform-pose *tf*
                                    :pose pregrasp-pose
                                    :target-frame "/torso_lift_link"))
               (grasp-pose (relative-grasp-pose
                            pose *grasp-offset*))
               (grasp-pose-tll
                 (tf:transform-pose *tf*
                                    :pose grasp-pose
                                    :target-frame "/torso_lift_link"))
               (grasp-solution
                 (first (get-ik;(get-constraint-aware-ik
                         side grasp-pose-tll))))
          ;; :allowed-collision-objects (list obj-desig)))))
          (unless grasp-solution
            (cpl:fail 'manipulation-pose-unreachable))
          (execute-grasp :pregrasp-pose pregrasp-pose-tll
                         :grasp-solution grasp-solution
                         :side side
                         :gripper-open-pos 0.1
                         :gripper-close-pos 0.01))))
    (loop for grasp-assignment in grasp-assignments
          for side = (slot-value grasp-assignment 'side)
          if (> (get-gripper-state side) 0.005)
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
                   :side side)))
          else
            do (cpl:fail 'cram-plan-failures:object-lost))))

(def-action-handler put-down (object-designator
                              location grasp-assignments obstacles)
  (declare (ignore obstacles))
  "Delegates the type of the put down action which suppose to be executed
for the currently type of grasped object."
  (clear-collision-objects)
  (roslisp:call-service "/collider_node/reset" "std_srvs/Empty")
  (let ((pair-count (length grasp-assignments)))
    (assert (> pair-count 0) ()
            "No arm/pose pairs specified during put-down.")
    ;; NOTE(winkler): The actual reasoning concerning object-in-hand
    ;; information and used grippers per grasped object part have been
    ;; done in the designator.lisp prolog patterns already. Here, we
    ;; just execute whatever result the reasoning yielded.
    (let* ((putdown-pose-pre (reference location))
           (putdown-pose
             (tf:pose->pose-stamped
              "/map"
              (roslisp:ros-time)
              (cl-transforms:transform-pose
               (tf:make-transform
                (tf:origin putdown-pose-pre)
                (tf:make-identity-rotation))
               (cl-transforms:transform-pose
                (tf:make-transform
                 (tf:make-identity-vector)
                 (tf:euler->quaternion :ax pi))
                (tf:make-pose-stamped
                 "/map"
                 (roslisp:ros-time)
                 (tf:make-identity-vector)
                 (tf:orientation putdown-pose-pre))))))
           (arm-poses
             (loop for grasp in grasp-assignments
                   for arm = (slot-value grasp 'side)
                   for pose = (slot-value grasp 'pose)
                   collect (cons arm (calculate-putdown-arm-pose
                                      putdown-pose pose)))))
      ;; Attach collision objects
      (loop for grasp in grasp-assignments
            for arm = (slot-value grasp 'side)
            for pose = (slot-value grasp 'pose)
            do (let* ((arm-link (ecase arm
                                  (:left "l_wrist_roll_link")
                                  (:right "r_wrist_roll_link")))
                      (effector-link (ecase arm
                                       (:left "l_end_effector")
                                       (:right "r_end_effector")))
                      (object-id (concatenate 'string "attached_object_" arm-link))
                      (msg (roslisp:make-message
                            "arm_navigation_msgs/AttachedCollisionObject"
                            (stamp header object) (roslisp:ros-time)
                            (frame_id header object) (tf:frame-id pose)
                            (link_name) arm-link
                            (id object) object-id
                            (padding object) 0.1
                            (operation operation object) 0
                            (shapes object) (vector
                                             (roslisp:make-message
                                              "arm_navigation_msgs/Shape"
                                              (type) 2
                                              (dimensions) (vector 0.1 (* 3 (abs (tf:z (tf:origin pose)))))))
                            (poses object) (vector
                                            (tf:pose->msg
                                             (cl-transforms:transform-pose
                                              (tf:pose->transform pose)
                                              (tf:make-pose
                                               (tf:make-3d-vector
                                                0.0 0.0 (tf:z
                                                         (tf:origin pose)))
                                               (tf:euler->quaternion)))))
                            (touch_links) (vector effector-link))))
                 (roslisp:publish
                  (roslisp:advertise
                   "/attached_collision_object"
                   "arm_navigation_msgs/AttachedCollisionObject")
                  msg)))
      (cpl:par-loop (arm-pose arm-poses)
        (let ((arm (car arm-pose))
              (pose (cdr arm-pose)))
          (tf:wait-for-transform
           *tf*
           :source-frame (tf:frame-id pose)
           :target-frame "/torso_lift_link"
           :time (roslisp:ros-time))
          (let* ((pre-putdown-pose (relative-grasp-pose
                                    pose *pre-putdown-offset*))
                 (pre-putdown-pose-tll
                   (tf:transform-pose *tf*
                                      :pose pre-putdown-pose
                                      :target-frame "/torso_lift_link"))
                 (putdown-pose (relative-grasp-pose
                                pose *putdown-offset*))
                 (putdown-pose-tll
                   (tf:transform-pose *tf*
                                      :pose putdown-pose
                                      :target-frame "/torso_lift_link"))
                 (putdown-solution
                   (first (get-ik arm putdown-pose-tll)))
                 (unhand-pose (relative-grasp-pose pose *grasp-offset*))
                 (unhand-pose-tll
                   (tf:transform-pose *tf*
                                      :pose unhand-pose
                                      :target-frame "/torso_lift_link"))
                 (unhand-solution (first (get-ik arm unhand-pose-tll))))
            (roslisp:publish (roslisp:advertise "/preputdownpose"
                                                "geometry_msgs/PoseStamped")
                             (tf:pose-stamped->msg pre-putdown-pose-tll))
            (unless (and putdown-solution unhand-solution)
              (cpl:fail 'manipulation-pose-unreachable))
            (execute-putdown :pre-putdown-pose pre-putdown-pose-tll
                             :putdown-solution putdown-solution
                             :unhand-solution unhand-solution
                             :side arm
                             :gripper-open-pos 0.1))))
      ;; Remove the attached collision objects
      (loop for grasp in grasp-assignments
            for arm = (slot-value grasp 'side)
            for pose = (slot-value grasp 'pose)
            do (let* ((arm-link (ecase arm
                                  (:left "l_wrist_roll_link")
                                  (:right "r_wrist_roll_link")))
                      (object-id (concatenate 'string "attached_object_" arm-link))
                      (msg (roslisp:make-message
                            "arm_navigation_msgs/AttachedCollisionObject"
                            (stamp header object) (roslisp:ros-time)
                            (frame_id header object) (tf:frame-id pose)
                            (link_name) arm-link
                            (id object) object-id
                            (padding object) 0.1
                            (operation operation object) 1)))
                 (roslisp:publish
                  (roslisp:advertise
                   "/attached_collision_object"
                   "arm_navigation_msgs/AttachedCollisionObject")
                  msg)))
      (loop for grasp-assignment in grasp-assignments
            for arm = (slot-value grasp-assignment 'side)
            do (with-vars-strictly-bound (?link-name)
                   (lazy-car
                    (prolog
                     `(cram-manipulation-knowledge:end-effector-link
                       ,arm ?link-name)))
                 (plan-knowledge:on-event
                  (make-instance
                   'plan-knowledge:object-detached
                   :object object-designator
                   :link ?link-name
                   :side arm)))))))

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
