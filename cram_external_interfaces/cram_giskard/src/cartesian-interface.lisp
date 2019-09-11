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

(defun make-giskard-cartesian-action-goal (left-pose right-pose
                                           pose-base-frame left-tool-frame right-tool-frame
                                           collision-mode
                                           &key
                                             collision-object-b collision-object-b-link
                                             collision-object-a
                                             move-the-ass)
  (declare (type (or null cl-transforms-stamped:pose-stamped) left-pose right-pose)
           (type string pose-base-frame left-tool-frame right-tool-frame))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal ;; :plan_only
                              :plan_and_execute
                              )
   :cmd_seq (vector
             (roslisp:make-message
              'giskard_msgs-msg:movecmd
              ;; THIS STUFF HAS A STATE
              ;; RESET THE STATE EXPLICITLY IF YOU WANT A NON CART MOVEMENT AFTER THIS
              :constraints
              (vector (roslisp:make-message
                       'giskard_msgs-msg:constraint
                       :type
                       "UpdateGodMap"
                       :parameter_value_pair
                       (let ((stream (make-string-output-stream)))
                         (yason:encode
                          (alexandria:alist-hash-table
                           `(("updates"
                              .
                              ,(alexandria:alist-hash-table
                                `(("rosparam"
                                   .
                                   ,(alexandria:alist-hash-table
                                     `(("joint_weights"
                                        .
                                        ,(alexandria:alist-hash-table
                                          `(("odom_x_joint" . ,(if move-the-ass 0.0001 1.0))
                                            ("odom_y_joint" . ,(if move-the-ass 0.0001 1.0))
                                            ("odom_z_joint" . ,(if move-the-ass 0.0001 1.0)))))))))
                                :test #'equal))))
                          stream)
                         (get-output-stream-string stream))))
              :cartesian_constraints
              (map 'vector #'identity
                   (remove nil
                           (list
                            (when left-pose
                              (roslisp:make-message
                               'giskard_msgs-msg:cartesianconstraint
                               :type (roslisp:symbol-code
                                      'giskard_msgs-msg:cartesianconstraint
                                      :translation_3d)
                               :root_link pose-base-frame
                               :tip_link left-tool-frame
                               :goal (cl-transforms-stamped:to-msg left-pose)))
                            (when left-pose
                              (roslisp:make-message
                               'giskard_msgs-msg:cartesianconstraint
                               :type (roslisp:symbol-code
                                      'giskard_msgs-msg:cartesianconstraint
                                      :rotation_3d)
                               :root_link pose-base-frame
                               :tip_link left-tool-frame
                               :goal (cl-transforms-stamped:to-msg left-pose)))
                            (when right-pose
                              (roslisp:make-message
                               'giskard_msgs-msg:cartesianconstraint
                               :type (roslisp:symbol-code
                                      'giskard_msgs-msg:cartesianconstraint
                                      :translation_3d)
                               :root_link pose-base-frame
                               :tip_link right-tool-frame
                               :goal (cl-transforms-stamped:to-msg right-pose)))
                            (when right-pose
                              (roslisp:make-message
                               'giskard_msgs-msg:cartesianconstraint
                               :type (roslisp:symbol-code
                                      'giskard_msgs-msg:cartesianconstraint
                                      :rotation_3d)
                               :root_link pose-base-frame
                               :tip_link right-tool-frame
                               :goal (cl-transforms-stamped:to-msg right-pose))))))
              :collisions
              (case collision-mode
                (:allow-all
                 (vector (roslisp:make-message
                          'giskard_msgs-msg:collisionentry
                          :type (roslisp:symbol-code
                                 'giskard_msgs-msg:collisionentry
                                 :allow_all_collisions))))
                (:avoid-all
                 (vector (roslisp:make-message
                          'giskard_msgs-msg:collisionentry
                          :type (roslisp:symbol-code
                                 'giskard_msgs-msg:collisionentry
                                 :avoid_all_collisions)
                          :min_dist 0.1)))
                (:allow-hand
                 (vector (roslisp:make-message
                          'giskard_msgs-msg:collisionentry
                          :type (roslisp:symbol-code
                                 'giskard_msgs-msg:collisionentry
                                 :avoid_all_collisions)
                          :min_dist 0.05)
                         (roslisp:make-message
                          'giskard_msgs-msg:collisionentry
                          :type (roslisp:symbol-code
                                 'giskard_msgs-msg:collisionentry
                                 :allow_collision)
                          :robot_links (apply
                                        #'vector
                                        (append
                                         (when left-pose
                                           (cut:var-value
                                            '?hand-links
                                            (car (prolog:prolog
                                                  `(and (rob-int:robot ?robot)
                                                        (rob-int:hand-links ?robot :left
                                                                            ?hand-links))))))
                                         (when right-pose
                                           (cut:var-value
                                            '?hand-links
                                            (car (prolog:prolog
                                                  `(and (rob-int:robot ?robot)
                                                        (rob-int:hand-links ?robot :right
                                                                            ?hand-links))))))))
                          :body_b (if collision-object-b
                                      (roslisp-utilities:rosify-underscores-lisp-name
                                       collision-object-b)
                                      (roslisp:symbol-code
                                            'giskard_msgs-msg:collisionentry
                                            :all))
                          :link_bs (vector (roslisp:symbol-code
                                            'giskard_msgs-msg:collisionentry
                                            :all))
                          ;; (if collision-object-b-link
                          ;;     (vector (roslisp-utilities:rosify-underscores-lisp-name
                          ;;              collision-object-b-link))
                          ;;     (vector (roslisp:symbol-code
                          ;;              'giskard_msgs-msg:collisionentry
                          ;;              :all)))
                          )
                         (roslisp:make-message
                          'giskard_msgs-msg:collisionentry
                          :type (roslisp:symbol-code
                                 'giskard_msgs-msg:collisionentry
                                 :allow_collision)
                          :robot_links (apply
                                        #'vector
                                        (append
                                         (when left-pose
                                           (cut:var-value
                                            '?hand-links
                                            (car (prolog:prolog
                                                  `(and (rob-int:robot ?robot)
                                                        (rob-int:hand-links ?robot :left
                                                                            ?hand-links))))))
                                         (when right-pose
                                           (cut:var-value
                                            '?hand-links
                                            (car (prolog:prolog
                                                  `(and (rob-int:robot ?robot)
                                                        (rob-int:hand-links ?robot :right
                                                                            ?hand-links))))))))
                          :body_b (roslisp-utilities:rosify-underscores-lisp-name
                                   :kitchen)
                          :link_bs (vector (roslisp:symbol-code
                                            'giskard_msgs-msg:collisionentry
                                            :all)))))
                (:allow-attached
                 (vector
                  (roslisp:make-message
                   'giskard_msgs-msg:collisionentry
                   :type (roslisp:symbol-code
                          'giskard_msgs-msg:collisionentry
                          :avoid_all_collisions)
                   :min_dist 0.02)
                  (roslisp:make-message
                   'giskard_msgs-msg:collisionentry
                   :type (roslisp:symbol-code
                          'giskard_msgs-msg:collisionentry
                          :allow_collision)
                   :robot_links (if collision-object-a
                                    (vector (roslisp-utilities:rosify-underscores-lisp-name
                                             collision-object-a))
                                    (vector (roslisp:symbol-code
                                             'giskard_msgs-msg:collisionentry
                                             :all))) ; collision-object-a = attached-obj
                   :body_b "kitchen";; (roslisp-utilities:rosify-underscores-lisp-name
                   ;;  collision-object-b-link)
                   :link_bs (if collision-object-b-link
                                (vector (roslisp-utilities:rosify-underscores-lisp-name
                                         collision-object-b-link))
                                (vector (roslisp:symbol-code
                                         'giskard_msgs-msg:collisionentry
                                         :all))))))
                (t
                 (vector (roslisp:make-message
                          'giskard_msgs-msg:collisionentry
                          :type (roslisp:symbol-code
                                 'giskard_msgs-msg:collisionentry
                                 :avoid_all_collisions)
                          :min_dist 0.1))))))))

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
    (return-from ensure-giskard-cartesian-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (pr2-ll giskard-cart) "Giskard action timed out."))
  (when (eql status :aborted)
    (roslisp:ros-warn (pr2-ll giskard-cart) "Giskard action aborted! With result ~a" result)
    ;; (cpl:fail 'common-fail:manipulation-goal-not-reached
    ;;           :description "Giskard did not converge to goal because of collision")
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
                                        (pose-base-frame ;; cram-tf:*robot-base-frame*
                                         cram-tf:*odom-frame*)
                                        (left-tool-frame cram-tf:*robot-left-tool-frame*)
                                        (right-tool-frame cram-tf:*robot-right-tool-frame*)
                                        (convergence-delta-xy *giskard-convergence-delta-xy*)
                                        (convergence-delta-theta *giskard-convergence-delta-theta*))
  (declare (type (or null cl-transforms-stamped:pose-stamped) goal-pose-left goal-pose-right)
           (type (or null number) action-timeout convergence-delta-xy convergence-delta-theta)
           (type (or null string) pose-base-frame left-tool-frame right-tool-frame))
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
                         :move-the-ass move-the-ass)))
              (actionlib-client:call-simple-action-client
               'giskard-action
               :action-goal goal
               :action-timeout action-timeout))
          (ensure-giskard-cartesian-goal-reached result status goal-pose-left goal-pose-right
                                                 left-tool-frame right-tool-frame
                                                 convergence-delta-xy convergence-delta-theta)
          (values result status)))
      (roslisp:ros-info (pr2-ll giskard-cart) "Got an empty goal...")))









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
