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
(defparameter *prefer-base-low-cost* 0.001)
(defparameter *avoid-collisions-distance* 0.10 "In cm, not used atm")
(defparameter *unmovable-joint-weight* 9001)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; UTILS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-constraints-vector (&rest entries)
  (if (every #'listp entries)
      (apply #'vector (alexandria:flatten entries))
      (apply #'vector (remove NIL entries))))

(defun make-giskard-goal (&key constraints joint-constraints cartesian-constraints
                            collisions (goal-type :plan_and_execute))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal goal-type)
   :cmd_seq (vector
             (roslisp:make-message
              'giskard_msgs-msg:movecmd
              :constraints (make-constraints-vector constraints)
              :cartesian_constraints (make-constraints-vector cartesian-constraints)
              :joint_constraints (make-constraints-vector joint-constraints)
              :collisions (make-constraints-vector collisions)))))

(defun cram-name-list->ros-frame-vector (cram-names-list)
  "@artnie used this function but actually other CRAM users probably don't need it..."
  (if cram-names-list
      (make-constraints-vector
       (mapcar (lambda (symbol)
                 (roslisp-utilities:rosify-underscores-lisp-name symbol))
               (if (listp cram-names-list)
                   cram-names-list
                   (list cram-names-list))))
      (vector (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all))))

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

(defun make-prefer-base-constraint (&optional (base-weight
                                               *prefer-base-low-cost*))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "UpdateGodMap"
   :parameter_value_pair
   (alist->json-string
    `(("updates"
       . (("rosparam"
           . (("general_options"
               . (("joint_weights"
                   . (("odom_x_joint" . ,base-weight)
                      ("odom_y_joint" . ,base-weight)
                      ("odom_z_joint" . ,base-weight)))))))))))))

(defun make-align-planes-constraint (root-frame tip-frame root-vector tip-vector)
  (declare (type string root-frame tip-frame)
           (type cl-transforms-stamped:vector-stamped root-vector tip-vector))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "AlignPlanes"
   :parameter_value_pair
   (alist->json-string
    `(("root" . ,root-frame)
      ("tip" . ,tip-frame)
      ("root_normal" . ,(to-hash-table root-vector))
      ("tip_normal" . ,(to-hash-table tip-vector))))))

(defun make-align-planes-tool-frame-constraint (arm root-vector tip-vector)
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
     cram-tf:*odom-frame* tool-frame root-vector tip-vector)))

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
    `(("root" . ,root-frame)
      ("tip" . ,tip-frame)
      ("goal_point" . ,(to-hash-table
                        (cram-tf:pose-stamped->point-stamped
                         goal-pose)))
      ,@(when pointing-vector
          `(("pointing_axis" . ,(to-hash-table pointing-vector))))))))

(defun make-open-or-close-constraint (open-or-close arm handle-link goal-joint-state)
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
      `(("object_name" . ,(roslisp-utilities:rosify-underscores-lisp-name
                           (rob-int:get-environment-name)))
        ("tip" . ,tool-frame)
        ("handle_link" . ,(roslisp-utilities:rosify-underscores-lisp-name
                           handle-link))
        ,@(when goal-joint-state
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
         . ,(to-hash-table tip-grasp-axis))
        ("bar_axis"
         . ,(to-hash-table bar-axis))
        ("bar_center"
         . ,(to-hash-table bar-center))
        ("tip"
         . ,tool-frame)
        ("bar_length"
         . ,bar-length)
        ("root"
         . ,root-link))))))

(defun make-cartesian-constraint (root-frame tip-frame goal-pose max-velocity)
  (declare (type string root-frame tip-frame)
           (type cl-transforms-stamped:pose-stamped goal-pose)
           (type number max-velocity))
  (list
   (roslisp:make-message
    'giskard_msgs-msg:constraint
    :type
    "CartesianPosition"
    :parameter_value_pair
    (alist->json-string
     `(("root_link" . ,root-frame)
       ("tip_link" . ,tip-frame)
       ("goal" . ,(to-hash-table goal-pose))
       ("max_velocity" . ,max-velocity))))
   (roslisp:make-message
    'giskard_msgs-msg:constraint
    :type
    "CartesianOrientationSlerp"
    :parameter_value_pair
    (alist->json-string
     `(("root_link" . ,root-frame)
       ("tip_link" . ,tip-frame)
       ("goal" . ,(to-hash-table goal-pose))
       ;; ("max_velocity" . ,max-velocity) ; can be used, but very experimental
       )))))

(defun make-joint-constraint (joint-state weights)
  (declare (type list joint-state weights))
  "`joint-state' is a list of two elements: (joint-names joint-positions).
`weights' is a list of the same length as `joint-names' and `joint-positions'."
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
                                         &optional (weight *unmovable-joint-weight*))
  (make-joint-constraint
   (list joint-names
         (joints:joint-positions joint-names))
   (make-list (length joint-names) :initial-element weight )))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; NON-JSON CONSTRAINTS ;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-simple-cartesian-constraint (root-link tip-link pose-stamped)
  (declare (type string root-link tip-link)
           (type cl-transforms-stamped:pose-stamped pose-stamped))
  (list (roslisp:make-message
         'giskard_msgs-msg:cartesianconstraint
         :type (roslisp:symbol-code
                'giskard_msgs-msg:cartesianconstraint
                :translation_3d)
         :root_link root-link
         :tip_link tip-link
         :goal (cl-transforms-stamped:to-msg pose-stamped))
        (roslisp:make-message
         'giskard_msgs-msg:cartesianconstraint
         :type (roslisp:symbol-code
                'giskard_msgs-msg:cartesianconstraint
                :rotation_3d)
         :root_link root-link
         :tip_link tip-link
         :goal (cl-transforms-stamped:to-msg pose-stamped))))

(defun make-simple-joint-constraint (joint-state)
  (declare (type list joint-state))
  "`joint-state' is a list of two elements: (joint-names joint-positions)."
  (roslisp:make-message
   'giskard_msgs-msg:jointconstraint
   :type (roslisp:symbol-code
          'giskard_msgs-msg:jointconstraint
          :joint)
   :goal_state (roslisp:make-message
                'sensor_msgs-msg:jointstate
                :name (apply #'vector (first joint-state))
                :position (apply #'vector (second joint-state)))))

(defun make-current-joint-state-constraint (arms)
  (let* ((joint-names-and-positions-lists
           (mapcar #'get-arm-joint-names-and-positions-list arms))
         (joint-names
           (mapcan #'first joint-names-and-positions-lists))
         (joint-positions
           (mapcan #'second joint-names-and-positions-lists)))
    (make-simple-joint-constraint (list joint-names joint-positions))))

(defun make-gripper-joint-state-constraint (arm joint-angle)
  (let ((gripper-joint
          (cut:var-value
           '?joint
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:gripper-joint ?robot ,arm ?joint)))))))
    (when (cut:is-var gripper-joint)
      (error "[giskard] Robot gripper joint was not defined."))
    (make-simple-joint-constraint `((,gripper-joint) (,joint-angle)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; COLLISIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-allow-all-collision ()
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :allow_all_collisions)))

(defun make-avoid-all-collision (&optional (minimal-distance
                                            *avoid-collisions-distance*))
  (declare (type number minimal-distance))
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :avoid_all_collisions)
   :min_dist minimal-distance))

(defun make-allow-robot-links-collision (links body-b &optional body-b-link)
  (declare (type list links)
           (type (or null keyword) body-b body-b-link))
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :allow_collision)
   :robot_links (make-constraints-vector links)
   :body_b (if body-b
               (roslisp-utilities:rosify-underscores-lisp-name body-b)
               (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all))
   :link_bs (vector
             (if body-b-link
                 (roslisp-utilities:rosify-underscores-lisp-name body-b-link)
                 (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all)))))

(defun make-allow-arm-collision (arms body-b &optional body-b-link)
  (declare (type list arms)
           (type (or null keyword) body-b body-b-link))
  (let ((arm-links
          (mapcan (lambda (arm)
                    (cut:var-value
                     '?arm-links
                     (car
                      (prolog:prolog
                       `(and (rob-int:robot ?robot)
                             (rob-int:arm-links ?robot ,arm ?arm-links))))))
                  arms)))
   (make-allow-robot-links-collision arm-links body-b body-b-link)))

(defun make-allow-hand-collision (arms body-b &optional body-b-link)
  (declare (type list arms)
           (type (or null keyword) body-b body-b-link))
  (let ((hand-links
          (mapcan (lambda (hand)
                    (cut:var-value
                     '?hand-links
                     (car
                      (prolog:prolog
                       `(and (rob-int:robot ?robot)
                             (rob-int:hand-links ?robot ,hand ?hand-links))))))
                  arms)))
   (make-allow-robot-links-collision hand-links body-b body-b-link)))

(defun make-allow-fingers-collision (arms body-b &optional body-b-link)
  (declare (type list arms)
           (type (or null keyword) body-b body-b-link))
  (let ((finger-links
          (mapcan (lambda (arm)
                    (cut:var-value
                     '?finger-links
                     (car
                      (prolog:prolog
                       `(and (rob-int:robot ?robot)
                             (rob-int:hand-links ?robot ,arm ?hand-links)
                             (setof ?finger-link
                                    (and (member ?finger-link ?hand-links)
                                         (rob-int:hand-finger-link
                                          ?robot ,arm ?finger-link))
                                    ?finger-links))))))
                  arms)))
    (make-allow-robot-links-collision finger-links body-b body-b-link)))

(defun make-allow-attached-collision (object-name environment-link)
  (declare (type (or symbol null) object-name)
           (type (or string null) environment-link))
  "`object-name' is the name of the attached object."
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :allow_collision)
   :robot_links (vector
                 (if object-name
                     (roslisp-utilities:rosify-underscores-lisp-name object-name)
                     (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all)))
   :body_b (roslisp-utilities:rosify-underscores-lisp-name
            (rob-int:get-environment-name))
   :link_bs (vector
             (if environment-link
                 (roslisp-utilities:rosify-underscores-lisp-name environment-link)
                 (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all)))))
