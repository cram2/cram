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

(in-package :giskard)

(defparameter *giskard-convergence-delta-xy* 0.1 ;; 0.005
  "in meters")
(defparameter *giskard-convergence-delta-theta* 0.5 ;; 0.1
  "in radiants, about 6 degrees")
(defparameter *move-base* t)
(defparameter *max-velocity* 0.1)

(defun make-giskard-cartesian-action-goal (left-pose right-pose
                                           pose-base-frame left-tool-frame right-tool-frame
                                           collision-mode
                                           &key
                                             collision-object-b collision-object-b-link
                                             collision-object-a
                                             ;; move-the-ass
                                             constrained-joints
                                             max-velocity)
  (declare (type (or null cl-transforms-stamped:pose-stamped) left-pose right-pose)
           (type string pose-base-frame left-tool-frame right-tool-frame))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal
                              ;; :plan_only
                              :plan_and_execute
                              )
   :cmd_seq (vector
             (roslisp:make-message
              'giskard_msgs-msg:movecmd
              ;; THIS STUFF HAS A STATE
              ;; RESET THE STATE EXPLICITLY IF YOU WANT A NON CART MOVEMENT AFTER THIS
              :constraints
              (apply #'vector
                     (remove nil
                             (append (coerce (constraint-jointposition constrained-joints) 'list)
                                     (constraint-cartesian-2 left-pose
                                                             left-tool-frame
                                                             pose-base-frame
                                                             *max-velocity*))))

              ;; :cartesian_constraints
              ;; (apply
              ;;  #'vector
              ;;  (remove
              ;;   nil
              ;;   (append
              ;;    (when left-pose
              ;;      (constraint-cartesian left-pose left-tool-frame pose-base-frame))
              ;;    (when right-pose
              ;;      (constraint-cartesian right-pose right-tool-frame pose-base-frame)))))
              :collisions
              (case collision-mode
                (:allow-all
                 (vector (constraint-collision-allow-all)))
                (:avoid-all
                 (vector (constraint-collision-avoid-all)))
                (:allow-hand
                 (vector
                  (constraint-collision-avoid-all :min-dist 0.05)
                  (constraint-collision-allow-hand left-pose right-pose collision-object-b
                                                   :link-bs collision-object-b-link)
                  (constraint-collision-allow-hand left-pose right-pose :kitchen)))
                (:allow-fingers
                 (vector
                  (constraint-collision-avoid-all :min-dist 0.02)
                  (constraint-collision-allow-fingers left-pose right-pose collision-object-b
                                                      :link-bs collision-object-b-link)
                  (constraint-collision-allow-hand left-pose right-pose :kitchen)))
                (:allow-attached
                 (vector (constraint-collision-avoid-all :min-dist 0.02)
                         (constraint-collision-allow-attached collision-object-a
                                                              collision-object-b
                                                              collision-object-b-link)))
                (t
                 (vector (constraint-collision-avoid-all :min-dist 0.1))))))))

(defun ensure-giskard-cartesian-input-parameters (frame left-pose right-pose)
  (values (when left-pose
            (cram-tf:ensure-pose-in-frame left-pose frame))
          (when right-pose
            (cram-tf:ensure-pose-in-frame right-pose frame))))

(defun ensure-giskard-cartesian-goal-reached (result status goal-position-left goal-position-right
                                              goal-frame-left goal-frame-right
                                              convergence-delta-xy convergence-delta-theta)
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Giskard action preempted with result ~a" result)
    (cpl:fail 'common-fail:manipulation-goal-not-reached
              :description "Giskard did not converge to goal because of preemtion")
    (return-from ensure-giskard-cartesian-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (pr2-ll giskard-cart) "Giskard action timed out."))
  (when (eql status :aborted)
    (roslisp:ros-warn (pr2-ll giskard-cart) "Giskard action aborted! With result ~a" result)
    (cpl:fail 'common-fail:manipulation-goal-not-reached
              :description "Giskard did not converge to goal because of collision")
    )
  (when goal-position-left
    (unless (cram-tf:tf-frame-converged goal-frame-left goal-position-left
                                        convergence-delta-xy convergence-delta-theta)
      (cpl:fail 'common-fail:manipulation-goal-not-reached
                :description (format nil "Giskard did not converge to goal:
~a should have been at ~a with delta-xy of ~a and delta-angle of ~a."
                                     goal-frame-left goal-position-left
                                     convergence-delta-xy convergence-delta-theta))))
  (when goal-position-right
    (unless (cram-tf:tf-frame-converged goal-frame-right goal-position-right
                                        convergence-delta-xy convergence-delta-theta)
      (cpl:fail 'common-fail:manipulation-goal-not-reached
                :description (format nil "Giskard did not converge to goal:
~a should have been at ~a with delta-xy of ~a and delta-angle of ~a."
                                     goal-frame-right goal-position-right    
                                     convergence-delta-xy convergence-delta-theta)))))

(defun call-giskard-cartesian-action (&key
                                        goal-pose-left goal-pose-right action-timeout
                                        collision-mode collision-object-b collision-object-b-link
                                        collision-object-a
                                        move-the-ass
                                        constraints
                                        (pose-base-frame ;; cram-tf:*robot-base-frame*
                                         (if *move-base*
                                             cram-tf:*odom-frame*
                                             cram-tf:*robot-base-frame*))
                                        (left-tool-frame cram-tf:*robot-left-tool-frame*)
                                        (right-tool-frame cram-tf:*robot-right-tool-frame*)
                                        (convergence-delta-xy *giskard-convergence-delta-xy*)
                                        (convergence-delta-theta *giskard-convergence-delta-theta*))
  (declare (type (or null cl-transforms-stamped:pose-stamped) goal-pose-left goal-pose-right)
           (type (or null number) action-timeout convergence-delta-xy convergence-delta-theta)
           (type (or null string) pose-base-frame left-tool-frame right-tool-frame))
  ;; (break)
  "When `collision-mode' allow specific collision:
`collision-object-b' is the object name for which colision is allowed.
"
  (if (or goal-pose-left goal-pose-right)
      (multiple-value-bind (goal-pose-left goal-pose-right)
          (ensure-giskard-cartesian-input-parameters pose-base-frame goal-pose-left goal-pose-right)
        (cram-tf:visualize-marker (list goal-pose-left goal-pose-right) :r-g-b-list '(1 0 1))
        (multiple-value-bind (result status)
            (let ((goal (make-giskard-cartesian-action-goal
                         goal-pose-left goal-pose-right
                         pose-base-frame left-tool-frame right-tool-frame
                         collision-mode
                         :collision-object-b collision-object-b
                         :collision-object-b-link collision-object-b-link
                         :collision-object-a collision-object-a
                         ;; :move-the-ass move-the-ass
                         :constrained-joints constraints
                         )))
              (actionlib-client:call-simple-action-client
               'giskard-action
               :action-goal goal
               :action-timeout (or action-timeout 120.0)))
          (ensure-giskard-cartesian-goal-reached result status goal-pose-left goal-pose-right
                                                 left-tool-frame right-tool-frame
                                                 convergence-delta-xy convergence-delta-theta)
          (values result status)
          ;; return the joint state, which is our observation
          (joints:full-joint-states-as-hash-table)
          ))
      ;; return NIL as observation if the goal is empty
      (and (roslisp:ros-info (pr2-ll giskard-cart) "Got an empty goal...")
           NIL)))









;; header: 
;;   seq: 17
;;   stamp: 
;;     secs: 1560439219
;;     nsecs: 637718915
;;   frame_id: ''
;; goal_id: 
;;   stamp: 
;;     secs: 1560439219
;;     nsecs: 637681961
;;   id: "/giskard_interactive_marker-17-1560439219.638"
;; goal: 
;;   type: 1
;;   cmd_seq: 
;;     - 
;;       constraints: []
;;       joint_constraints: []
;;       cartesian_constraints: 
;;         - 
;;           type: "CartesianPosition"
;;           root_link: "odom"
;;           tip_link: "base_footprint"
;;           goal: 
;;             header: 
;;               seq: 0
;;               stamp: 
;;                 secs: 0
;;                 nsecs:         0
;;               frame_id: "base_footprint"
;;             pose: 
;;               position: 
;;                 x: 2.50292941928e-08
;;                 y: 0.0
;;                 z: 0.209769845009
;;               orientation: 
;;                 x: 0.0
;;                 y: 0.0
;;                 z: 0.0
;;                 w: 1.0
;;         - 
;;           type: "CartesianOrientationSlerp"
;;           root_link: "odom"
;;           tip_link: "base_footprint"
;;           goal: 
;;             header: 
;;               seq: 0
;;               stamp: 
;;                 secs: 0
;;                 nsecs:         0
;;               frame_id: "base_footprint"
;;             pose: 
;;               position: 
;;                 x: 2.50292941928e-08
;;                 y: 0.0
;;                 z: 0.209769845009
;;               orientation: 
;;                 x: 0.0
;;                 y: 0.0
;;                 z: 0.0
;;                 w: 1.0
;;       collisions: 
;;         - 
;;           type: 1
;;           min_dist: 0.0
;;           robot_links: ['']
;;           body_b: "pr2"
;;           link_bs: ['']
