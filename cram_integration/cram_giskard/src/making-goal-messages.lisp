;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defparameter *avoid-joint-limits-percentage* 40)
(defparameter *prefer-base-low-cost* 0.0001)
(defparameter *allow-base-high-cost* 0.01)
(defparameter *avoid-collisions-distance* 0.10 "In m.")
(defparameter *unmovable-joint-weight* 9001)
(defparameter *collision-avoidance-hint-threshold* 0.2 "In m.")
(defparameter *collision-avoidance-hint-spring-offset* 0.1 "In m.")
(defparameter *collision-avoidance-hint-velocity* 1.0 "In m/s")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; UTILS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-constraints-vector (&rest entries)
  (if (vectorp (car entries))
      (car entries)
      (if (every #'listp entries)
          (apply #'vector (alexandria:flatten entries))
          (apply #'vector (remove NIL entries)))))

(defun make-giskard-goal (&key
                            constraints joint-constraints cartesian-constraints
                            collisions
                            (goal-type :plan_and_execute_and_cut_off_shaking))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal goal-type)
   :cmd_seq (vector
             (roslisp:make-message
              'giskard_msgs-msg:movecmd
              :constraints (make-constraints-vector (append constraints
                                                            (list joint-constraints)
                                                            (list cartesian-constraints)))
              :collisions (make-constraints-vector collisions)))))

(defun make-giskard-goal-multiple (&key
                                     all-constraints
                                     joint-constraints
                                     cartesian-constraints
                                     collisions
                                     (goal-type
                                      :plan_and_execute_and_cut_off_shaking))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal goal-type)
   :cmd_seq (map
             'vector
             (lambda (constraints)
               (roslisp:make-message
                'giskard_msgs-msg:movecmd
                :constraints (make-constraints-vector constraints
                                                      cartesian-constraints
                                                      joint-constraints)
                :collisions (make-constraints-vector collisions)))
             all-constraints)))


(defun get-arm-joint-names-and-positions-list (arm &optional joint-states)
  "Returns a list of two elements: (joint-names joint-positions)"
  (if joint-states
      (list (mapcar #'first joint-states)
            (mapcar #'second joint-states))
      (let ((joint-names
              (cut:var-value
               '?joints
               (cut:lazy-car
                (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:arm-joints ?robot ,arm ?joints)))))))
        (unless (cut:is-var joint-names)
          (list joint-names
                (joints:joint-positions joint-names))))))

(defun get-neck-joint-names-and-positions-list (&optional joint-states)
  "Returns a list of two elements: (joint-names joint-positions)"
  (if joint-states
      (list (mapcar #'first joint-states)
            (mapcar #'second joint-states))
      (let ((joint-names
              (cut:var-value
               '?joints
               (cut:lazy-car
                (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:robot-neck-joints ?robot . ?joints)))))))
        (unless (cut:is-var joint-names)
          (list joint-names
                (joints:joint-positions joint-names))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; JSON CONSTRAINTS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-avoid-joint-limits-constraint (&optional (percentage
                                                      *avoid-joint-limits-percentage*))
  (declare (type number percentage))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "AvoidJointLimits"
   :parameter_value_pair
   (alist->json-string
    `(("percentage" . ,percentage)))))


(defun make-prefer-base-constraint (&key
                                      (base-weight *prefer-base-low-cost*)
                                      do-not-rotate)
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "OverwriteWeights"
   :parameter_value_pair
   (alist->json-string
    `(("updates"
       . (("1"
           . (("odom_x_joint" . ,base-weight)
              ("odom_y_joint" . ,base-weight)
              ("odom_z_joint" . ,(if do-not-rotate
                                     *unmovable-joint-weight*
                                     base-weight))))
          ("2"
           . (("odom_x_joint" . ,base-weight)
              ("odom_y_joint" . ,base-weight)
              ("odom_z_joint" . ,(if do-not-rotate
                                     *unmovable-joint-weight*
                                     base-weight))))
          ("3"
           . (("odom_x_joint" . ,base-weight)
              ("odom_y_joint" . ,base-weight)
              ("odom_z_joint" . ,(if do-not-rotate
                                     *unmovable-joint-weight*
                                     base-weight))))))))))


(defun make-align-planes-constraint (root-frame tip-frame root-vector tip-vector
                                     &key avoid-collisions-not-much)
  (declare (type string root-frame tip-frame)
           (type cl-transforms-stamped:vector-stamped root-vector tip-vector))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "AlignPlanes"
   :parameter_value_pair
   (alist->json-string
    `(("root_link" . ,root-frame)
      ("tip_link" . ,tip-frame)
      ("root_normal"
       . (("message_type" . "geometry_msgs/Vector3Stamped")
          ("message" . ,(to-hash-table root-vector))))
      ("tip_normal"
       . (("message_type" . "geometry_msgs/Vector3Stamped")
          ("message" . ,(to-hash-table tip-vector))))
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))

(defun make-align-planes-tool-frame-constraint (arm root-vector tip-vector
                                                &key avoid-collisions-not-much)
  (declare (type keyword arm)
           (type cl-transforms-stamped:vector-stamped root-vector tip-vector))
  (let ((tool-frame
          (cut:var-value
           '?frame
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:robot-tool-frame ?robot ,arm ?frame)))))))
    (when (cut:is-var tool-frame)
      (error "[giskard] Tool frame was not defined."))
    (make-align-planes-constraint
     cram-tf:*odom-frame* tool-frame root-vector tip-vector
     :avoid-collisions-not-much avoid-collisions-not-much)))

(defun make-pointing-constraint (root-frame tip-frame goal-pose
                                 &optional pointing-vector)
  (declare (type string root-frame tip-frame)
           (type cl-transforms-stamped:pose-stamped goal-pose)
           (type (or cl-transforms-stamped:vector-stamped null) pointing-vector))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "Pointing"
   :parameter_value_pair
   (alist->json-string
    `(("root_link" . ,root-frame)
      ("tip_link" . ,tip-frame)
      ("goal_point"
       . (("message_type" . "geometry_msgs/PointStamped")
          ("message" . ,(to-hash-table
                         (cram-tf:pose-stamped->point-stamped
                          goal-pose)))))
      ,@(when pointing-vector
          `(("pointing_axis" . ,(to-hash-table pointing-vector))))))))

(defun make-head-pointing-constraint (goal-pose)
  (declare (type cl-transforms-stamped:pose-stamped goal-pose))
  (let ((camera-frame
          (cut:var-value
           '?camera-frame
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:camera-frame ?robot ?camera-frame)))))))
    (when (cut:is-var camera-frame)
      (error "[giskard] Camera frame was not defined."))
    (make-pointing-constraint
     (cl-transforms-stamped:frame-id goal-pose)
     camera-frame
     goal-pose)))

(defun make-head-pointing-at-hand-constraint (arm)
  (declare (type keyword arm))
  (let ((tool-frame
          (cut:var-value
           '?frame
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:robot-tool-frame ?robot ,arm ?frame)
                       (rob-int:camera-frame ?robot ?camera-frame)))))))
    (when (cut:is-var tool-frame)
      (error "[giskard] Tool frame was not defined."))
    (make-head-pointing-constraint
     (cl-transforms-stamped:pose->pose-stamped
      tool-frame 0.0
      (cl-transforms:make-identity-pose)))))

(defun make-open-or-close-constraint (open-or-close arm handle-link &optional goal-joint-state)
  (declare (type keyword open-or-close arm)
           (type keyword handle-link)
           (type (or number null) goal-joint-state))
  (let ((tool-frame
          (cut:var-value
           '?frame
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:robot-tool-frame ?robot ,arm ?frame)))))))
    (when (cut:is-var tool-frame)
      (error "[giskard] Tool frame was not defined."))
    (roslisp:make-message
     'giskard_msgs-msg:constraint
     :type
     (roslisp-utilities:rosify-lisp-name open-or-close)
     :parameter_value_pair
     (alist->json-string
      `(("tip_link" . ,tool-frame)
        ("environment_link" . ,(roslisp-utilities:rosify-underscores-lisp-name
                                handle-link))
        ,@(when (and (eq open-or-close :open) goal-joint-state)
            `(("goal_joint_state" . ,goal-joint-state))))))))

(defun make-grasp-bar-constraint (arm root-link
                                  tip-grasp-axis
                                  bar-axis bar-center bar-length)
  (let ((tool-frame
          (cut:var-value
           '?frame
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:robot-tool-frame ?robot ,arm ?frame)))))))
    (when (cut:is-var tool-frame)
      (error "[giskard] Tool frame was not defined."))
    (roslisp:make-message
     'giskard_msgs-msg:constraint
     :type
     "GraspBar"
     :parameter_value_pair
     (alist->json-string
      `(("tip_grasp_axis"
         . (("message_type" . "geometry_msgs/Vector3Stamped")
            ("message" . ,(to-hash-table tip-grasp-axis))))
        ("bar_axis"
         . (("message_type" . "geometry_msgs/Vector3Stamped")
            ("message" . ,(to-hash-table bar-axis))))
        ("bar_center"
         . (("message_type" . "geometry_msgs/PointStamped")
            ("message" . ,(to-hash-table bar-center))))
        ("tip_link"
         . ,tool-frame)
        ("bar_length"
         . ,bar-length)
        ("root_link"
         . ,root-link))))))

(defun make-cartesian-constraint (root-frame tip-frame goal-pose
                                  &key max-velocity avoid-collisions-much)
  (declare (type string root-frame tip-frame)
           (type cl-transforms-stamped:pose-stamped goal-pose)
           (type (or number null) max-velocity)
           (type boolean avoid-collisions-much))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "CartesianPose"
   :parameter_value_pair
   (alist->json-string
    `(("root_link" . ,root-frame)
      ("tip_link" . ,tip-frame)
      ("goal_pose"
       . (("message_type" . "geometry_msgs/PoseStamped")
          ("message" . ,(to-hash-table goal-pose))))
      ,@(when max-velocity
          `(("max_linear_velocity" . ,max-velocity)))
      ,@(if avoid-collisions-much
          `(("weight" . ,(roslisp-msg-protocol:symbol-code
                          'giskard_msgs-msg:constraint
                          :weight_below_ca
                          )))
          `(("weight" . ,(roslisp-msg-protocol:symbol-code
                          'giskard_msgs-msg:constraint
                          :weight_above_ca ; that's the default weight anyway
                          ))))))))

(defun make-joint-constraint (joint-state &optional weights)
"`joint-state' is a list of two elements: (joint-names joint-positions).
`weights' is a list of the same length as `joint-names' and `joint-positions',
a keyword, a number or NIL."
  (declare (type list joint-state)
           (type (or list number keyword) weights))
  (if (and weights (listp weights))
      (make-joint-constraints-in-multiple-msgs joint-state weights)
      (make-joint-constraint-list-in-one-msg joint-state weights)))

(defun make-joint-constraint-list-in-one-msg (joint-state &optional weights)
  (declare (type list joint-state)
           (type (or null number keyword) weights))
  "Creates a JointPositionList msg with one fixed weight for all joints.
`joint-state' is a list of two elements: (joint-names joint-positions).
`weights' is a list of the same length as `joint-names' and `joint-positions'."
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "JointPositionList"
   :parameter_value_pair
   (alist->json-string
    `(("goal_state" . ,(mapcar (lambda (joint-name joint-position)
                                 `(,joint-name . ,joint-position))
                               (first joint-state)
                               (second joint-state)))
      ,@(when weights
          `(("weight" . ,(case weights
                           (:avoid-collisions-not-much
                            (roslisp-msg-protocol:symbol-code
                             'giskard_msgs-msg:constraint
                             :weight_above_ca))
                           (t
                            weights)))))))))

(defun make-joint-constraints-in-multiple-msgs (joint-state weights)
  (declare (type list joint-state)
           (type list weights))
  "`joint-state' is a list of two elements: (joint-names joint-positions).
`weights' is a list of the same length as `joint-names' and `joint-positions'."
  (unless (eq (length (first joint-state)) (length weights))
    (error "Making joint constraint failed. Too few weights ~a for joints ~a."
           weights (first joint-state)))
  (mapcar (lambda (joint-name joint-position weight)
            (roslisp:make-message
             'giskard_msgs-msg:constraint
             :type
             "JointPosition"
             :parameter_value_pair
             (alist->json-string
              `(("joint_name" . ,joint-name)
                ("goal" . ,joint-position)
                ("weight" . ,weight)))))
          (first joint-state)
          (second joint-state)
          weights))

(defun make-unmovable-joints-constraint (joint-names
                                         &optional
                                           (weight *unmovable-joint-weight*))
  (declare (type list joint-names)
           (type (or number null) weight))
  (make-joint-constraint
   (list joint-names
         (joints:joint-positions joint-names))
   weight))

(defun make-base-collision-avoidance-hint-constraint (environment-link
                                                      vector
                                                      &optional
                                                        (threshold
                                                         *collision-avoidance-hint-threshold*)
                                                        (spring-offset
                                                         *collision-avoidance-hint-spring-offset*)
                                                        (max-velocity
                                                         *collision-avoidance-hint-velocity*))
  (declare (type string environment-link)
           (type cl-transforms-stamped:vector-stamped vector)
           (type number threshold spring-offset max-velocity))
  (let ((base-link
          (cut:var-value
           '?link
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:robot-base-link ?robot ?link)))))))
    (when (cut:is-var base-link)
      (error "[giskard] Robot base link was not defined."))
    (roslisp:make-message
     'giskard_msgs-msg:constraint
     :type
     "CollisionAvoidanceHint"
     :parameter_value_pair
     (alist->json-string
      `(("tip_link" . ,base-link)
        ("avoidance_hint"
         . (("message_type" . "geometry_msgs/Vector3Stamped")
            ("message" . ,(to-hash-table vector))))
        ("object_group" . ,(roslisp-utilities:rosify-underscores-lisp-name
                           (rob-int:get-environment-name)))
        ("object_link_name" . ,environment-link)
        ;; ("root_link" . "map")
        ("max_linear_velocity" . ,max-velocity)
        ("max_threshold" . ,threshold)
        ("spring_threshold" . ,(+ threshold spring-offset))
        ("weight" . ,(roslisp-msg-protocol:symbol-code
                      'giskard_msgs-msg:constraint
                      :weight_max)))))))

(defun make-base-velocity-constraint (max-linear-velocity max-angular-velocity)
  (declare (type number max-linear-velocity max-angular-velocity))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "CartesianVelocityLimit"
   :parameter_value_pair
   (alist->json-string
    `(("root_link" . ,cram-tf:*odom-frame*)
      ("tip_link" . ,cram-tf:*robot-base-frame*)
      ("max_linear_velocity" . ,max-linear-velocity)
      ("max_angular_velocity" . ,max-angular-velocity)))))

(defun make-ee-velocity-constraint (arm
                                    max-linear-velocity max-angular-velocity)
  (declare (type number max-linear-velocity max-angular-velocity)
           (type keyword arm))
  (let ((link
          (cut:var-value
           '?link
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:end-effector-link ?robot ,arm ?link)))))))
    (when (cut:is-var link)
      (error "[giskard] Robot ee link was not defined."))
    (roslisp:make-message
     'giskard_msgs-msg:constraint
     :type
     "CartesianVelocityLimit"
     :parameter_value_pair
     (alist->json-string
      `(("root_link" . ,cram-tf:*odom-frame*)
        ("tip_link" . ,link)
        ("max_linear_velocity" . ,max-linear-velocity)
        ("max_angular_velocity" . ,max-angular-velocity)
        ("hard" . 0))))))

(defun make-current-joint-state-constraint (arms)
  (let* ((joint-names-and-positions-lists
           (mapcar #'get-arm-joint-names-and-positions-list arms))
         (joint-names
           (mapcan #'first joint-names-and-positions-lists))
         (joint-positions
           (mapcan #'second joint-names-and-positions-lists)))
    (make-joint-constraint (list joint-names joint-positions))))

(defun make-gripper-joint-state-constraint (arm joint-angle)
  (let ((gripper-joints
          (mapcar (lambda (bindings)
                    (cut:var-value '?joint bindings))
                  (cut:force-ll
                   (prolog:prolog
                    `(and (rob-int:robot ?robot)
                          (rob-int:gripper-joint ?robot ,arm ?joint)))))))
    (unless gripper-joints
      (error "[giskard] Robot gripper joint was not defined."))
    (make-joint-constraint
     (list gripper-joints
           (make-list (length gripper-joints) :initial-element joint-angle)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; COLLISIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-allow-all-collision ()
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :allow_collision)
   :group1 (roslisp-utilities:rosify-underscores-lisp-name
            (rob-int:get-robot-name))
   :group2 (roslisp:symbol-code
            'giskard_msgs-msg:collisionentry
            :all)))

(defun make-allow-self-collision ()
  "Only for debugging purposes."
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :allow_collision)
   :group1 (roslisp-utilities:rosify-underscores-lisp-name
            (rob-int:get-robot-name))
   :group2 (roslisp-utilities:rosify-underscores-lisp-name
            (rob-int:get-robot-name))))

(defun make-avoid-all-collision (&optional (minimal-distance *avoid-collisions-distance*))
  (declare (type number minimal-distance))
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :avoid_collision)
   :distance minimal-distance
   :group1 (roslisp-utilities:rosify-underscores-lisp-name
            (rob-int:get-robot-name))
   :group2 (roslisp:symbol-code
            'giskard_msgs-msg:collisionentry
            :all)))


(defun make-allow-robot-links-collision (group1 group2)
  (declare (type string group1)
           (type (or null string) group2))
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :allow_collision)
   :group1 group1
   :group2 (or group2
               (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all))))


(defun find-root-links (link-names)
  (declare (type list link-names))
  "Takes a list of links and reduces it to only contain the root link(s)."
  (let* ((robot-links-table (cl-urdf:links rob-int:*robot-urdf*))
         (link-parent-alist
           (mapcar (lambda (link-name)
                     `(,link-name
                       . ,(cl-urdf:name
                           (cl-urdf:parent
                            (cl-urdf:from-joint
                             (gethash link-name robot-links-table))))))
                   link-names)))
    ;; remove all whose parent is in `link-names'
    (mapcar #'car
            (remove-if (alexandria:rcurry #'member
                                          (mapcar #'car link-parent-alist)
                                          :test #'string=)
                       link-parent-alist
                       :key #'cdr))))

(defun make-allow-arm-collision (arms body-b &optional body-b-link)
  (declare (type list arms)
           (type (or null keyword) body-b body-b-link))
  "Allows collision between all given arms and body-b-link if defined, or just body-b."
  (let* ((arms-roots
           (find-root-links
            (mapcan (lambda (arm)
                        (cut:var-value
                         '?arm-links
                         (car
                          (prolog:prolog
                           `(and (rob-int:robot ?robot)
                                 (rob-int:arm-links ?robot ,arm ?arm-links))))))
                    arms)))
         collision-group2)
    (when body-b
      (setf collision-group2 (roslisp-utilities:rosify-underscores-lisp-name
                              body-b)))
    (when body-b-link
      (register-group :root-link
                      (roslisp-utilities:rosify-underscores-lisp-name
                       body-b-link)
                      :parent-group-name
                      (roslisp-utilities:rosify-underscores-lisp-name
                       (or body-b (rob-int:get-environment-name))))
      (setf collision-group2 (roslisp-utilities:rosify-underscores-lisp-name
                              body-b-link)))
    (loop for root in arms-roots
          do (register-group :root-link root
                             :parent-group-name
                             (roslisp-utilities:rosify-underscores-lisp-name
                              (rob-int:get-robot-name)))
          collect (make-allow-robot-links-collision root collision-group2))))

(defun make-allow-hand-collision (arms body-b &optional body-b-link)
  (declare (type list arms)
           (type (or null keyword) body-b body-b-link))
  "Allows collision between all given hands (in arms) and body-b-link if defined, or just body-b."
  (let ((hand-roots
          (find-root-links
           (mapcan (lambda (hand)
                     (cut:var-value
                      '?hand-links
                      (car
                       (prolog:prolog
                        `(and (rob-int:robot ?robot)
                              (rob-int:hand-links ?robot ,hand ?hand-links))))))
                   arms)))
        collision-group2)
    (when body-b
      (setf collision-group2 (roslisp-utilities:rosify-underscores-lisp-name
                              body-b)))
    (when body-b-link
      (register-group :root-link
                      (roslisp-utilities:rosify-underscores-lisp-name
                       body-b-link)
                      :parent-group-name
                      (roslisp-utilities:rosify-underscores-lisp-name
                       (or body-b (rob-int:get-environment-name))))
      (setf collision-group2 (roslisp-utilities:rosify-underscores-lisp-name
                              body-b-link)))
    (loop for root in hand-roots
          do (register-group :root-link root
                             :parent-group-name
                             (roslisp-utilities:rosify-underscores-lisp-name
                              (rob-int:get-robot-name)))
          collect (make-allow-robot-links-collision root collision-group2))))

(defun make-allow-fingers-collision (arms body-b &optional body-b-link)
  (declare (type list arms)
           (type (or null keyword) body-b body-b-link))
  "Allows collision between all given hand's fingers and body-b-link if defined, or just body-b."
  (let ((finger-roots
          (find-root-links
           (mapcan (lambda (arm)
                     (cut:var-value
                      '?finger-links
                      (car
                       (prolog:prolog
                        `(and (rob-int:robot ?robot)
                              (rob-int:hand-links ?robot ,arm ?hand-links)
                              (prolog:setof
                               ?finger-link
                               (and (member ?finger-link ?hand-links)
                                    (rob-int:hand-finger-link
                                     ?robot ,arm ?finger-link))
                               ?finger-links))))))
                   arms)))
        collision-group2)
    (when body-b ; if body-b exists, set as group2
      (setf collision-group2 (roslisp-utilities:rosify-underscores-lisp-name
                              body-b)))
    (when body-b-link ; if specific link is defined, register and use as group2
      (register-group :root-link
                      (roslisp-utilities:rosify-underscores-lisp-name
                       body-b-link)
                      :parent-group-name
                      (roslisp-utilities:rosify-underscores-lisp-name
                       (or body-b (rob-int:get-environment-name))))
      (setf collision-group2 (roslisp-utilities:rosify-underscores-lisp-name
                              body-b-link)))
    (loop for root in finger-roots
          do (register-group :root-link root
                             :parent-group-name
                             (roslisp-utilities:rosify-underscores-lisp-name
                              (rob-int:get-robot-name)))
          collect (make-allow-robot-links-collision root collision-group2))))

(defun make-allow-collision-with-environment-attachments ()
  "Allows collision towards all objects attached to the environment."
  (mapcar (lambda (attachment-info)
            (make-allow-robot-links-collision
             (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all)
             (roslisp-utilities:rosify-underscores-lisp-name (car attachment-info))))
          (btr:attached-objects (btr:get-environment-object))))
