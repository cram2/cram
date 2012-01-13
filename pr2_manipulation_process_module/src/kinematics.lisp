;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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
;;;

(in-package :pr2-manip-pm)

(defparameter *ik-left-ns* "/pr2_left_arm_kinematics")
(defparameter *ik-right-ns* "/pr2_right_arm_kinematics")

(defvar *joint-state* nil)

(defun seq-member (item sequence)
  (some (lambda (s)
          (equal item s))
        sequence))

(defun lazy-cross-product (&rest sets)
  (if (car sets)
      (let ((cp (apply #'lazy-cross-product (cdr sets))))
        (lazy-mapcan (lambda (e)
                       (lazy-mapcar (lambda (x) (cons e x)) cp))
                     (car sets)))
      (list nil)))

(defun get-joint-position (state joint-name)
  (roslisp:with-fields (name position) state
    (loop for n across name
          for p across position
          when (equal joint-name n)
            do (return p))))

(defun get-joint-names (side)
  (roslisp:with-fields ((joint-names (joint_names kinematic_solver_info)))
      (cpl-impl:without-scheduling
        (roslisp:call-service
         (concatenate
          'string
          (ecase side
            (:right *ik-right-ns*)
            (:left *ik-left-ns*))
          "/get_ik_solver_info")
         "kinematics_msgs/GetKinematicSolverInfo"))
    joint-names))

(defun make-seed-states (side joint-names &optional (steps 3))
  (flet ((init ()
           (let ((current (make-hash-table :test 'equal))
                 (lower (make-hash-table :test 'equal))
                 (upper (make-hash-table :test 'equal))
                 (joint-state *joint-state*))
             (roslisp:with-fields ((joint-names (joint_names kinematic_solver_info))
                                   (limits (limits kinematic_solver_info)))
                 (cpl-impl:without-scheduling
                   (roslisp:call-service
                    (concatenate
                     'string
                     (ecase side
                       (:right *ik-right-ns*)
                       (:left *ik-left-ns*))
                     "/get_ik_solver_info")
                    "kinematics_msgs/GetKinematicSolverInfo"))
               (map nil (lambda (limit joint-name)
                          (roslisp:with-fields ((min min_position)
                                                (max max_position))
                              limit
                            (setf (gethash joint-name lower) min)
                            (setf (gethash joint-name upper) max)
                            (setf (gethash joint-name current)
                                  (get-joint-position joint-state joint-name))))
                    limits joint-names)
               (values joint-names current lower upper)))))
    (multiple-value-bind (names current lower-limits upper-limits)
        (init)
      (lazy-mapcar (lambda (joint-states)
                     (roslisp:make-msg
                      "sensor_msgs/JointState"
                      (stamp header) 0
                      name names
                      position (reverse
                                (map 'vector #'identity joint-states))
                      velocity (make-array (length names)
                                           :element-type 'float
                                           :initial-element 0.0)
                      effort (make-array (length names)
                                         :element-type 'float
                                         :initial-element 0.0)))
                   (apply #'lazy-cross-product
                          (reverse
                           (loop for name across names collecting
                             (if (seq-member name joint-names)
                                 (cons (gethash name current)
                                       (loop for i from 0 below steps collecting
                                         (+ (gethash name lower-limits)
                                            (* (- steps i 1)
                                               (/ (- (gethash name upper-limits)
                                                     (gethash name lower-limits))
                                                  (- steps 1))))))
                                 (list (gethash name current))))))))))

(defun ik->trajectory (ik-result &key (duration 5.0))
  (declare (type kinematics_msgs-srv:getpositionik-response ik-result))
  (roslisp:with-fields ((solution-names (name joint_state solution))
                        (solution-positions (position joint_state solution))
                        (error-code (val error_code)))
      ik-result
    (when (eql error-code 1)
      (roslisp:make-message
       "trajectory_msgs/JointTrajectory"
       (stamp header) (roslisp:ros-time)
       joint_names solution-names
       points (vector
               (roslisp:make-message
                "trajectory_msgs/JointTrajectoryPoint"
                positions solution-positions
                velocities (map 'vector #'identity
                                (make-list (length solution-positions)
                                           :initial-element 0.0))
                accelerations (map 'vector #'identity
                                   (make-list (length solution-positions)
                                              :initial-element 0.0))
                time_from_start duration))))))

(defun remove-trajectory-joints (joints trajectory &key invert)
  "Removes (or keeps) only the joints that are specified in
  `joints'. If `invert' is NIL, the named joints are removed,
  otherwise, they are kept."
  (roslisp:with-fields ((stamp (stamp header))
                        (joint-names joint_names)
                        (points points))
      trajectory
    (if (not invert)
        (roslisp:make-message
         "trajectory_msgs/JointTrajectory"
         (stamp header) stamp
         joint_names (remove-if (lambda (name)
                                  (seq-member name joints))
                                joint-names)
         points (map 'vector
                     (lambda (point)
                       (roslisp:with-fields (positions
                                             velocities
                                             accelerations
                                             time_from_start)
                           point
                         (roslisp:make-message
                          "trajectory_msgs/JointTrajectoryPoint"
                          positions (map 'vector #'identity
                                         (loop for n across joint-names
                                               for p across positions
                                               unless (seq-member n joints)
                                                 collecting p))
                          velocities (map 'vector #'identity
                                          (loop for n across joint-names
                                                for p across velocities
                                                unless (seq-member n joints)
                                                  collecting p))
                          accelerations (map 'vector #'identity
                                             (loop for n across joint-names
                                                   for p across accelerations
                                                   unless (seq-member n joints)
                                                     collecting p))
                          time_from_start time_from_start)))
                     points))
        (roslisp:make-message
         "trajectory_msgs/JointTrajectory"
         (stamp header) stamp
         joint_names (map 'vector #'identity
                          (loop for n across joint-names
                                when (seq-member n joints)
                                  collecting n))
         points (map 'vector
                     (lambda (point)
                       (roslisp:with-fields (positions time_from_start)
                           point
                         (roslisp:make-message
                          "trajectory_msgs/JointTrajectoryPoint"
                          positions (map 'vector #'identity
                                         (loop for n across joint-names
                                               for p across positions
                                               when (seq-member n joints)
                                                 collecting p))
                          time_from_start time_from_start)))
                     points)))))

(defun merge-trajectories (velocity trajectory &rest trajectories)
  (if trajectories
      (roslisp:with-fields ((stamp (stamp header))
                            (joint-names joint_names)
                            (points-1 points))
          trajectory
        (roslisp:with-fields ((points-2 points)) (car trajectories)
          (let ((last (unless (cdr trajectories)
                        (elt points-2 (1- (length points-2))))))
            (apply
             #'merge-trajectories
             velocity
             (roslisp:make-message
              "trajectory_msgs/JointTrajectory"
              (stamp header) stamp
              joint_names joint-names
              points (let ((time 0)
                           (current-time))
                       (map
                        'vector
                        (lambda (pt)
                          (roslisp:with-fields (positions time_from_start) pt
                            (when (eq pt (elt points-2 0))
                              (incf time current-time))
                            (setf current-time (+ time time_from_start))
                            (prog1
                                (if (not (eq pt last))
                                    (roslisp:make-message
                                     "trajectory_msgs/JointTrajectoryPoint"
                                     positions positions
                                     velocities (map 'vector #'identity
                                                     (make-list (length joint-names)
                                                                :initial-element velocity))
                                     accelerations (map 'vector #'identity
                                                        (make-list (length joint-names)
                                                                   :initial-element 0.0))
                                     time_from_start current-time)
                                    (roslisp:make-message
                                     "trajectory_msgs/JointTrajectoryPoint"
                                     positions positions
                                     velocities (map 'vector #'identity
                                                     (make-list (length joint-names)
                                                                :initial-element 0.0))
                                     accelerations (map 'vector #'identity
                                                        (make-list (length joint-names)
                                                                   :initial-element 0.0))
                                     time_from_start current-time))
                              )))
                        (concatenate 'vector points-1 points-2))))
             (cdr trajectories)))))
      trajectory))

(defun get-ik (side pose &key
               (tool (cl-transforms:make-pose
                      (cl-transforms:make-3d-vector 0 0 0)
                      (cl-transforms:make-quaternion 0 0 0 1)))
               (max-tries 1)
               seed-state)
  (let ((seeds (append
                (when seed-state (list seed-state))
                (make-seed-states
                 side (remove "torso_lift_joint"
                              (get-joint-names side)
                              :test #'equal)))))
    (lazy-list ((seeds (if max-tries
                           (lazy-take max-tries seeds)
                           seeds)))
      (when seeds
        (let ((result (cpl-impl:without-scheduling
                        (roslisp:call-service
                         (concatenate
                          'string
                          (ecase side
                            (:right *ik-right-ns*)
                            (:left *ik-left-ns*))
                          "/get_ik")
                         'kinematics_msgs-srv:getpositionik
                       
                         :ik_request
                         (roslisp:make-msg
                          "kinematics_msgs/PositionIKRequest"
                          :ik_link_name (ecase side
                                          (:right "r_wrist_roll_link")
                                          (:left "l_wrist_roll_link"))
                          :pose_stamped (tf:pose-stamped->msg
                                         (calculate-grasp-pose pose :tool tool))
                          :ik_seed_state (roslisp:make-msg
                                          "arm_navigation_msgs/RobotState"
                                          joint_state (lazy-car seeds)))
                         :timeout 1.0))))
          (roslisp:with-fields ((error-code (val error_code)))
              result
            (if (eql error-code 1)
                (cont result (lazy-cdr seeds))
                (next (lazy-cdr seeds)))))))))

(defun get-constraint-aware-ik (side pose &key
                                (tool (cl-transforms:make-pose
                                       (cl-transforms:make-3d-vector 0 0 0)
                                       (cl-transforms:make-quaternion 0 0 0 1)))
                                allowed-collision-objects
                                (max-tries 1)
                                seed-state)
  (let ((seeds (append
                (when seed-state (list seed-state))
                (make-seed-states
                 side (remove "torso_lift_joint"
                              (get-joint-names side)
                              :test #'equal)))))
    (lazy-list ((seeds (if max-tries
                           (lazy-take max-tries seeds)
                           seeds)))
      (when seeds
        (let ((result (cpl-impl:without-scheduling
                        (roslisp:call-service
                         (concatenate
                          'string
                          (ecase side
                            (:right *ik-right-ns*)
                            (:left *ik-left-ns*))
                          "/get_constraint_aware_ik")
                         'kinematics_msgs-srv:getconstraintawarepositionik
                       
                         :ik_request
                         (roslisp:make-msg
                          "kinematics_msgs/PositionIKRequest"
                          :ik_link_name (ecase side
                                          (:right "r_wrist_roll_link")
                                          (:left "l_wrist_roll_link"))
                          :pose_stamped (tf:pose-stamped->msg
                                         (calculate-grasp-pose pose :tool tool))
                          :ik_seed_state (roslisp:make-msg
                                          "arm_navigation_msgs/RobotState"
                                          joint_state (lazy-car seeds)))
                         :ordered_collision_operations (make-collision-operations
                                                        side
                                                        (cons "\"attached\"" allowed-collision-objects))
                       
                         :timeout 3.0))))
          (roslisp:with-fields ((error-code (val error_code)))
              result
            (if (eql error-code 1)
                (cont result (lazy-cdr seeds))
                (next (lazy-cdr seeds)))))))))

(defun get-sgp-grasps (side obj)
  (ecase side
    (:right (roslisp:set-param "/grasp_pcd/sgp_config_param_start_side" 0.0))
    (:left (roslisp:set-param "/grasp_pcd/sgp_config_param_start_side" pi)))
  (let ((grasps (cpl-impl:without-scheduling
                  (roslisp:call-service
                   "/grasp_pcd/simple_grasp_planner" "sgp_srvs/sgp"
                   :query (jlo:partial-lo (perception-pm:object-jlo (reference obj)))))))
    (roslisp:with-fields (poses quals) grasps
      (lazy-mapcar
       (lambda (g)
         (tf:msg->pose (car g)))
       (sort (map 'list #'cons poses quals)
             #'> :key #'cdr)))))

(defun get-point-cluster-grasps (side obj)
  "Returns the (lazy) list of grasps calculated by the
  /plan_point_cluster_grasps service."
  (let ((obj-tf-inv (cl-transforms:transform-inv
                     (cl-transforms:reference-transform
                      (tf:transform-pose
                       *tf* :pose (designator-pose obj)
                       :target-frame "/base_footprint")))))
    (roslisp:with-fields ((error-code (value error_code))
                          (grasps grasps))
        (cpl-impl:without-scheduling
          (roslisp:call-service
           "/plan_point_cluster_grasp" 'object_manipulation_msgs-srv:graspplanning
           :arm_name (ecase side
                       (:left "left_arm")
                       (:right "right_arm"))
           :target (cop-obj->graspable-obj obj "/base_footprint")))
      (unless (= error-code 0)
        (error 'manipulation-failed
               :format-control "Couldn't find valid grasps"))
      (lazy-mapcar
       (lambda (g)
         (cl-transforms:transform->pose
          (cl-transforms:transform*
           obj-tf-inv
           (cl-transforms:reference-transform (tf:msg->pose (car g))))))
       (let ((result (sort (map 'list
                                (lambda (grasp)
                                  (roslisp:with-fields (grasp_pose success_probability)
                                      grasp
                                    (cons grasp_pose success_probability)))
                                grasps)
                           #'> :key #'cdr)))
         (prog1 result
           (roslisp:ros-info (pr2-manip process-module) "found ~a grasps~%" (length result))))))))

(defun calculate-tool-pose (grasp tool-length)
  (cl-transforms:transform->pose
   (cl-transforms:transform*
    (cl-transforms:make-transform
     (cl-transforms:make-3d-vector tool-length 0 0)
     (cl-transforms:make-quaternion 0 0 0 1))             
    (cl-transforms:transform-inv
     (cl-transforms:reference-transform grasp)))))

(defun calculate-grasp-pose (obj &key
                             (tool (cl-transforms:make-pose
                                    (cl-transforms:make-3d-vector 0 0 0)
                                    (cl-transforms:make-quaternion 0 0 0 1))))
  (let* ((pose (etypecase obj
                 (designator (desig:designator-pose obj))
                 (tf:pose-stamped obj)))
         (goal-trans (cl-transforms:transform*
                      (cl-transforms:reference-transform pose)
                      (cl-transforms:transform-inv
                       (cl-transforms:reference-transform tool)))))
    (tf:make-pose-stamped
     (tf:frame-id pose) (tf:stamp pose)
     (cl-transforms:translation goal-trans)
     (cl-transforms:rotation goal-trans))))

(defun calculate-put-down-pose (obj location)
  (let ((obj-loc (current-desig (desig-prop-value (current-desig obj) 'at))))
    (assert obj-loc () "Object ~a needs to have an `at' property"
            obj)
    (assert (and (eq (desig-prop-value obj-loc 'in) 'gripper)) ()
                 "Object ~a needs to be in the gripper" obj)
    (assert (desig-prop-value obj-loc 'pose) ()
            "Object ~a needs to have a `pose' property" obj)
    (assert (desig-prop-value obj-loc 'height) ()
            "Object ~a needs to have a `height' property" obj)
    (let* ((obj-in-gripper (desig-prop-value obj-loc 'pose))
           (obj-pose-height (desig-prop-value obj-loc 'height))
           (location-in-robot (tf:transform-pose
                               *tf* :pose (reference (current-desig location))
                               :target-frame "/base_footprint"))
           (goal-trans (cl-transforms:transform*
                        (cl-transforms:reference-transform
                         (tf:copy-pose-stamped
                          location-in-robot
                          :origin (cl-transforms:v+
                                   (cl-transforms:origin location-in-robot)
                                   (cl-transforms:make-3d-vector 0 0 obj-pose-height))))
                        (cl-transforms:transform-inv
                         (cl-transforms:reference-transform obj-in-gripper)))))
      (tf:make-pose-stamped
       "/base_footprint" (tf:stamp location-in-robot)
       (cl-transforms:translation goal-trans)
       (cl-transforms:rotation goal-trans)))))

(defun get-grasp-object-trajectory-points (side obj)
  (let ((grasps (get-sgp-grasps side obj))
        (pose (designator-pose obj)))
    (lazy-car
     (lazy-mapcan (lambda (grasp)
                    (list (merge-trajectories
                           1.0
                           (ik->trajectory
                            (lazy-car (get-ik
                                       side pose
                                       :tool (calculate-tool-pose grasp 0.30)
                                       :max-tries 30)))
                           (ik->trajectory
                            (lazy-car (get-ik
                                       side pose
                                       :tool (calculate-tool-pose grasp 0.15)
                                       :max-tries 30))))))
                  grasps))))

(defun grasp-orientation-valid (pose good bad)
  "Returns T if `pose' has an orientation that is closer to an
  orientation in `good' than to an orientation in `bad'. Returns NIL
  otherwise."
  (labels ((find-closest-angle (rot rotations &optional closest)
             (let ((angle (and rotations
                               (cl-transforms:angle-between-quaternions
                                rot (car rotations)))))
               (cond ((not rotations)
                      closest)
                     ((or (not closest) (< angle closest))
                      (find-closest-angle rot (cdr rotations) angle))
                     (t (find-closest-angle rot (cdr rotations) closest))))))
    (let ((pose-in-base (tf:transform-pose
                         *tf* :pose pose
                         :target-frame "/base_footprint")))
      (< (find-closest-angle (cl-transforms:orientation pose-in-base)
                             good)
         (find-closest-angle (cl-transforms:orientation pose-in-base)
                             bad)))))

(defun calculate-carry-orientation (obj side orientations)
  (declare (ignore orientations))
  (when obj
    (let* ((hand-orientation (cl-transforms:rotation
                              (tf:lookup-transform
                               *tf*  
                               :source-frame (ecase side
                                               (:right "r_wrist_roll_link")
                                               (:left "l_wrist_roll_link"))
                               :target-frame "/base_footprint"))))
      hand-orientation)))

(defun get-robot-state ()
  "Returns the current joint state of the robot"
  (roslisp:with-fields ((joint-state (joint_state robot_state)))
      (cpl-impl:without-scheduling
        (roslisp:call-service
         "/environment_server/get_robot_state"
         "arm_navigation_msgs/GetRobotState"))
    joint-state))

(defun get-gripper-state (side)
  (roslisp:with-fields (name position)
      (get-robot-state)
    (let ((idx (position (ecase side
                           (:right "r_gripper_joint")
                           (:left "l_gripper_joint"))
                         name :test #'equal)))
      (unless idx
        (error 'simple-error :format-control "Invalid robot state. Couldn't find gripper joint"))
      (elt position idx))))

(defun get-gripper-links (side)
  (roslisp:get-param (ecase side
                       (:right "/hand_description/right_arm/hand_touch_links")
                       (:left "/hand_description/left_arm/hand_touch_links"))))

(defun make-collision-operations (side &optional allowed-collision-objects)
  "Returns an instance of type
`arm_navigation_msgs/OrderedCollisionOperations' and allows
collisions beteen the gripper and
`collision-objects'. `collision-objects' is a list of either strings
or designators."
  (roslisp:make-msg
   "arm_navigation_msgs/OrderedCollisionOperations"
   collision_operations
   (map 'vector #'identity
        (mapcan
         (lambda (obj)
           (mapcar (lambda (gripper-link)
                     (roslisp:make-msg
                      "arm_navigation_msgs/CollisionOperation"
                      object1 gripper-link
                      object2 (etypecase obj
                                (string obj)
                                (object-designator (get-collision-object-name obj)))
                      operation 0
                      penetration_distance 0.1))
                   (get-gripper-links side)))
         allowed-collision-objects))))
