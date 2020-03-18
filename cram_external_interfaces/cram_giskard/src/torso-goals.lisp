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

(defparameter *torso-convergence-delta-joint* 0.01 "in meters")

(defun make-giskard-torso-action-goal (joint-angle)
  (declare (type number joint-angle))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal ;; :plan_only
                              :plan_and_execute
                              )
   :cmd_seq (vector
             (roslisp:make-message
              'giskard_msgs-msg:movecmd
              :joint_constraints
              (vector (roslisp:make-message
                       'giskard_msgs-msg:jointconstraint
                       :type (roslisp:symbol-code
                              'giskard_msgs-msg:jointconstraint
                              :joint)
                       :goal_state (roslisp:make-message
                                    'sensor_msgs-msg:jointstate
                                    :name (vector cram-tf:*robot-torso-joint*)
                                    :position (vector joint-angle))))))))

(defun ensure-torso-input-parameters (position)
  (declare (type (or keyword number) position))
  (let* ((bindings
           (car
            (prolog:prolog
             `(and (rob-int:robot ?robot)
                   (rob-int:robot-torso-link-joint ?robot ?_ ?joint)
                   (rob-int:joint-lower-limit ?robot ?joint ?lower)
                   (rob-int:joint-upper-limit ?robot ?joint ?upper)))))
         (lower-limit
           (+ (cut:var-value '?lower bindings) 0.01)) ; don't push the robot so hard
         (upper-limit
           (- (cut:var-value '?upper bindings) 0.01)) ; no don't do it
         (cropped-joint-angle
           (etypecase position
             (number (if (< position lower-limit)
                         lower-limit
                         (if (> position upper-limit)
                             upper-limit
                             position)))
             (keyword (ecase position
                        (:upper-limit upper-limit)
                        (:lower-limit lower-limit)
                        (:middle (/ (- upper-limit lower-limit) 2)))))))
    cropped-joint-angle))



(defun ensure-giskard-torso-goal-reached (result status goal convergence-delta)
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Giskard action preempted with result ~a" result)
    (return-from ensure-giskard-torso-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (pr2-ll giskard-cart) "Giskard action timed out."))
  (when (eql status :aborted)
    (roslisp:ros-warn (pr2-ll giskard-cart) "Giskard action aborted! With result ~a" result))
  (let ((current-position (car (joints:joint-positions (list cram-tf:*robot-torso-joint*)))))
    (unless (cram-tf:values-converged current-position goal convergence-delta)
      (cpl:fail 'common-fail:torso-goal-not-reached
                :description (format nil "Giskard did not converge to torso goal:~
                                                goal: ~a, current: ~a, delta: ~a."
                                     goal current-position convergence-delta)))))

(defun call-giskard-torso-action (&key
                                    goal-joint-state action-timeout
                                    (convergence-delta *torso-convergence-delta-joint*))
  (declare (type (or number keyword) goal-joint-state)
           (type (or null number) action-timeout convergence-delta))
  (let ((goal-joint-state (ensure-torso-input-parameters goal-joint-state)))
    (multiple-value-bind (result status)
        (let ((goal (make-giskard-torso-action-goal goal-joint-state)))
          (actionlib-client:call-simple-action-client
           'giskard-action
           :action-goal goal
           :action-timeout action-timeout))
      (ensure-giskard-torso-goal-reached result status goal-joint-state convergence-delta)
      (values result status)
      ;; return the joint state, which is our observation
      (joints:full-joint-states-as-hash-table))))









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
