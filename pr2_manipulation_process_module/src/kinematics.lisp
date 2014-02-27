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

(in-package :pr2-manipulation-process-module)

(defparameter *ik-left-ns* "/pr2_left_arm_kinematics")
(defparameter *ik-right-ns* "/pr2_right_arm_kinematics")

(defvar *joint-state* nil)

(defgeneric designator->graspable-object (designator reference-frame)
  (:documentation "Returns an instance of type
  `object_manipulation_msgs/GraspableObject' for the object designator
  `designator'. `reference-frame' is the frame the graspable object
  should be in."))

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

(defun get-arm-state (side)
  "Returns the state of one arm specified by `side', i.e. joint-names,
current joint-values, lower joint limits, and upper joint limits -each
as a vector. "
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
      (values joint-names current lower upper))))

(defun make-seed-states (side joint-names &optional (steps 3))
  "Creates a lazy list of seed states. `steps' indicates how many
steps should be used for going from joint angle minimum to
maximum. E.g 3 means that minimum, (mimimum + maximum) / 2 and maximum
are used for each joint."
  (multiple-value-bind (names current lower-limits upper-limits)
      (get-arm-state side)
    (lazy-mapcar
     (lambda (joint-states)
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
     (apply
      #'lazy-cross-product
      (reverse
       (loop for name across names
             collecting (if (seq-member name joint-names)
                            (cons
                             (gethash name current)
                             (loop for i from 0 below steps
                                   collecting (+ (gethash name lower-limits)
                                                 (* (- steps i 1)
                                                    (/ (- (gethash
                                                           name upper-limits)
                                                          (gethash
                                                           name lower-limits))
                                                       (- steps 1))))))
                            (list (gethash name current)))))))))

(defun calculate-put-down-pose (obj location)
  (let ((object-goal-transform (calc-goal-transform-for-picked-object obj location)))
    (let* ((obj-loc (current-desig (desig-prop-value (current-desig obj) 'at)))
           (obj-in-gripper (desig-prop-value obj-loc 'pose))
           (goal-trans (cl-transforms:transform*
                        object-goal-transform
                        (cl-transforms:transform-inv
                         (cl-transforms:reference-transform obj-in-gripper)))))
      (tf:make-pose-stamped
       "/base_footprint" (tf:stamp (reference (current-desig location)))
       (cl-transforms:translation goal-trans)
       (cl-transforms:rotation goal-trans)))))

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
  "Returns a top orientation when object has been grasped from the top
and a side orientation otherwise."
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
  "Returns the position of the gripper. 0 indicates a completely
closed gripper."
  (let ((joint-name (ecase side
                      (:right "r_gripper_joint")
                      (:left "l_gripper_joint"))))
    (cram-moveit:get-joint-value joint-name)))

(defun get-gripper-links (side)
  "Returns the names of the gripper's links."
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

(defun euclidean-distance-for-link (names-from positions-from names-to
                                    positions-to arm target-link)
  "Calculates and returns the euclidean distance of exactly one link
`targe-link' between two joint space configurations. The
configurations are given as vectors, each consisting of a `names' and
a `positions' vector. The `-from' and `-to' pairs are interchangable
here. If no target links are given, all available links will be
used. This function is a simplified interface function for
`euclidean-distance', which takes a vector of target links as
parameter and returns the distance for each of these."
  (cdr (assoc
        target-link
        (euclidean-distance names-from positions-from
                            names-to positions-to arm
                            :target-links (vector target-link))
        :test 'equal)))

(defun available-fk-links-for-arm (arm)
  (case arm
    (:left (list "l_shoulder_pan_link"
                 "l_shoulder_lift_link"
                 "l_upper_arm_roll_link"
                 "l_upper_arm_link"
                 "l_elbow_flex_link"
                 "l_forearm_roll_link"
                 "l_forearm_link"
                 "l_wrist_flex_link"
                 "l_wrist_roll_link"))
    (:right (list "r_shoulder_pan_link"
                  "r_shoulder_lift_link"
                  "r_upper_arm_roll_link"
                  "r_upper_arm_link"
                  "r_elbow_flex_link"
                  "r_forearm_roll_link"
                  "r_forearm_link"
                  "r_wrist_flex_link"
                  "r_wrist_roll_link"))))

(defun available-fk-links ()
  "Returns the usable forward kinematics links as returned by the
service `get_fk_solver_info'."
  (concatenate 'vector
               (available-fk-links-for-arm :left)
               (available-fk-links-for-arm :right)))

(defun available-fk-joints-for-arm (arm)
  (roslisp:with-fields (kinematic_solver_info)
      (roslisp:call-service
       (concatenate
        'string
        (ecase arm
          (:right *ik-right-ns*)
          (:left *ik-left-ns*))
        "/get_fk_solver_info")
       'kinematics_msgs-srv:getkinematicsolverinfo)
    (roslisp:with-fields (joint_names limits link_names)
        kinematic_solver_info
      (declare (ignore link_names limits))
      joint_names)))

(defun available-fk-joints ()
  "Returns the usable forward kinematics joints as returned by the
service `get_fk_solver_info'."
  (concatenate 'vector
               (available-fk-joints-for-arm :left)
               (available-fk-joints-for-arm :right)))

(defun resulting-fk-state (names positions)
  "Forms the resulting set of possible forward kinematics joints and
the joints given in the `names' vector. Returns a multiple bound value
list consisting of all FK joint names, the given positions in
`positions' (or 0 if the joint is not specified in `names') and two
vectors holding the value `0' for each joint's velocity and
acceleration."
  (let ((jts-names nil)
        (jts-positions nil)
        (jts-velocities nil)
        (jts-accels nil)
        (avail-jts (available-fk-joints)))
    (loop for joint across avail-jts
          do (let* ((pos (position joint names :test 'equal))
                    (val (cond (pos (elt positions pos))
                               (t 0.0))))
               (setf jts-names (concatenate 'vector
                                            jts-names
                                            (vector joint)))
               (setf jts-positions (concatenate 'vector
                                                jts-positions
                                                (vector val)))
               (setf jts-velocities (concatenate 'vector
                                                 jts-velocities
                                                 (vector 0)))
               (setf jts-accels (concatenate 'vector
                                             jts-accels
                                             (vector 0)))))
    (values jts-names jts-positions jts-velocities jts-accels)))

(defun get-positions-from-trajectory (trajectory &key (index 0))
  "Extracts the positions field of a given trajectory-point index from
a given trajectory. The result is a sequence and consists of the joint
angles in the same order as the names-field define in the same
trajectory."
  (roslisp:with-fields (points) trajectory
    (let ((point (elt points index)))
      (roslisp:with-fields (positions) point
        positions))))

(defun joint-state-distance (names-from positions-from names-to
                             positions-to)
  "Calculates the square summed difference between to joint-space
positions. Only named joint-states found in both sequence pairs are
used during the calculation."
  (let ((dist 0))
    (dotimes (n (length names-from))
      (let* ((name-from (elt names-from n))
             (position-to (position name-from
                                    names-to
                                    :test (lambda (name-from name-to)
                                            (equal
                                             name-from
                                             name-to)))))
        (when position-to
          (let ((pos-from (elt positions-from n))
                (pos-to (elt positions-to position-to)))
            (incf dist (expt (- pos-from pos-to) 2))))))
    dist))

(defun reaching-length (pose side &key constraint-aware
                                    calc-euclidean-distance
                                    euclidean-target-link
                                    allowed-collision-objects
                                    highlight-links)
  "Calculates the squared sum of all joint angle differences between
the current state of the robot and the joint state it would have after
reaching pose `pose' through calculating a trajectory via inverse
kinematics solution for side `side'. All intermediate trajectory
points between the starting joint-state and the final trajectory point
are taken into account. `nil' is returned when no inverse kinematics
solution could be found. Optionally, the constraint aware IK solver
service can be used by setting the parameter `constraint-aware' to
`t'. When `calc-euclidean-distance' is set to `t', the euclidean
distance is used. Otherwise, the (unweighted) quadratic joint-space
integral is calculated. Both methods may not be mixed as their scale
is fundamentally different."
  (let* ((wrist-frame (ecase side
                        (:left "l_wrist_roll_link")
                        (:right "r_wrist_roll_link")))
         (arm-group (ecase side
                      (:left "left_arm")
                      (:right "right_arm")))
         (pose-in-tll
           (moveit:ensure-pose-stamped-transformed
            pose "/torso_lift_link" :ros-time t)))
    (let ((state-0 (moveit:plan-link-movement
                    wrist-frame arm-group pose-in-tll
                    :touch-links
                    (links-for-arm-side side)
                    :allowed-collision-objects
                    allowed-collision-objects
                    :destination-validity-only t
                    :highlight-links highlight-links)))
      (when state-0
        (moveit:pose-distance wrist-frame pose)))))
