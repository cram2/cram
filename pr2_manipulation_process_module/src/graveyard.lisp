;;; NOTE(winkler): This file holds `old' functions that are not really
;;; used anymore. The code might be reusable, though. This file is not
;;; loaded by default.

(defparameter *pregrasp-offset-pose*
  (tf:make-pose
   (tf:make-3d-vector 0.20 0.0 0.0)
   (tf:euler->quaternion :az pi :ax (/ pi 2)))
  "Specifies the gripper pose relative to the respective handle
coordinate system (including it's origin and rotation) when going into
pregrasp.")
(defparameter *grasp-offset-pose*
  (tf:make-pose
   (tf:make-3d-vector 0.1 0.0 0.0)
   (tf:euler->quaternion :az pi :ax (/ pi 2)))
  "Specifies the gripper pose relative to the respective handle
coordinate system (including it's origin and rotation) when going into
grasp.")

(defun arms-handle-distances (arms handle-pose
                              &key
                                allowed-collision-objects
                                constraint-aware
                                (arms-offset-pose
                                 (tf:make-identity-pose))
                                highlight-links)
  "Calculates the distances for each arm given in the `arms' list with
respect to the handle `handle'. Only arms that can actually reach the
handle are included. The resulting list consists of entries of the
form `((\"with-offset\" . distance-with-offset) (\"without-offset\"
. distance-without-offset)' for each arm given in `arms'. If no arm
could reach the handle, `NIL' is returned."
  (flet ((apply-pose-offset (pose offset-pose)
           (cl-transforms:transform-pose
            (cl-transforms:pose->transform pose)
            offset-pose)))
    (loop for arm in arms
          for target-link = (ecase arm
                              (:left "l_wrist_roll_link")
                              (:right "r_wrist_roll_link"))
          for handle-pose-offsetted = (apply-pose-offset handle-pose
                                                         arms-offset-pose)
          for handle-pose-stamped = (tf:make-pose-stamped
                                     "/map"
                                     0.0
                                     (tf:origin handle-pose-offsetted)
                                     (tf:orientation handle-pose-offsetted))
          for distance = (reaching-length handle-pose-stamped arm
                                          :constraint-aware constraint-aware
                                          :calc-euclidean-distance nil
                                          :euclidean-target-link target-link
                                          :allowed-collision-objects
                                          allowed-collision-objects
                                          :highlight-links highlight-links)
          when distance
            collect (cons arm distance))))


(defun relative-pose-for-handle (obj handle &key relative-pose)
  (tf:wait-for-transform *tf*
                         :timeout 5.0
                         :time (ros-time)
                         :source-frame "map"
                         :target-frame "/map")
  (let* ((absolute-pose-map
           (absolute-handle
            obj handle
            :handle-offset-pose relative-pose))
         (absolute-pose (tf:transform-pose *tf*
                                           :pose absolute-pose-map
                                           :target-frame "/map")))
    absolute-pose))

(defun optimal-arm-handle-assignment (obj avail-arms avail-handles min-handles
                                      &key (max-handles
                                            (or (desig-prop-value
                                                 obj 'desig-props:max-handles)
                                                nil)))
  (ros-info (pr2 manip-pm) "Opening grippers")
  (dolist (arm avail-arms)
    (open-gripper arm))
  (ros-info (pr2 manip-pm) "Calculating optimal grasp: ~a arms, ~a handles (min ~a, max ~a)" (length avail-arms) (length avail-handles) min-handles max-handles)
  (let ((assigned-entities
          (entity-assignment
           (list
            (make-assignable-entity-list
             :entities avail-arms)
            (make-assignable-entity-list
             :entities avail-handles
             :min-assignments min-handles
             :max-assignments max-handles)))))
    (ros-info (pr2 manip-pm) "Entities assigned")
    (let ((valid-assignments
            (remove-if
             (lambda (assign)
               (not (cost-function-grasp-handle-ik-constraint-aware obj assign)))
             assigned-entities)))
      (ros-info (pr2 manip-pm) "Entities validated")
      (let ((sorted-valid-assignments
              (sort
               valid-assignments
               (lambda (assign-1 assign-2)
                 (let ((cost1 (cost-function-grasp-handle-ik-constraint-aware
                               obj assign-1))
                       (cost2 (cost-function-grasp-handle-ik-constraint-aware
                               obj assign-2)))
                   (cond ((and cost1 cost2)
                          (< cost1 cost2))
                         (cost1 cost1)
                         (cost2 cost2)))))))
        (ros-info (pr2 manip-pm) "Entities sorted")
        (let ((assignments (mapcar (lambda (arm handle-pair)
                                     (make-instance
                                      'grasp-assignment
                                      :pose (reference
                                             (desig-prop-value
                                              (cdr handle-pair) 'at))
                                      :side arm
                                      :handle-pair handle-pair))
                                   (first (first sorted-valid-assignments))
                                   (second (first sorted-valid-assignments)))))
          (ros-info
           (pr2 manip-pm) "Assignment complete. Count = ~a" (length assignments))
          (unless assignments
            (cpl:fail 'cram-plan-failures:manipulation-pose-unreachable))
          (ros-info (pr2 manip-pm) "Optimal grasp calculation complete")
          assignments)))))

(defun open-angle (object handle angle-deg)
  (let* ((angle-rad (* (/ angle-deg 180.0) pi))
         (object-pose (reference (desig-prop-value object 'desig-props:at)))
         (joint (desig-prop-value handle 'desig-props:joint))
         (joint-pose (reference (desig-prop-value joint 'desig-props:at)))
         (joint-axis (desig-prop-value joint 'desig-props:joint-axis))
         (handle-pose (reference (desig-prop-value handle 'desig-props:at))))
    (let* ((roll (* (tf:x joint-axis) angle-rad))
           (pitch (* (tf:y joint-axis) angle-rad))
           (yaw (* (tf:z joint-axis) angle-rad))
           (rot-q (tf:euler->quaternion :ax roll :ay pitch :az yaw)))
      (let* ((zeroed-handle-pose (tf:make-pose
                                  (tf:v- (tf:origin handle-pose)
                                         (tf:origin joint-pose))
                                  (tf:orientation handle-pose)))
             (rotated-handle-pose (cl-transforms:transform-pose
                                   (tf:make-transform
                                    (tf:make-identity-vector)
                                    rot-q)
                                   zeroed-handle-pose))
             (relative-handle-pose (cl-transforms:transform-pose
                                    (tf:make-transform
                                     (tf:origin joint-pose)
                                     (tf:make-identity-rotation))
                                    rotated-handle-pose))
             (absolute-handle-pose (cl-transforms:transform-pose
                                    (cl-transforms:pose->transform
                                     object-pose)
                                    relative-handle-pose)))
        (tf:pose->pose-stamped
         (tf:frame-id object-pose)
         (ros-time)
         absolute-handle-pose)))))

(defun highlight-ik (ik side &key highlight)
  (let ((links (when highlight (pr2-manip-pm::links-for-arm-side side))))
    (moveit:display-robot-state ik :highlight links)))

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

  (<- (action-desig ?desig (container-opened ?current-obj ?grasp-assignments
                                             ?angle))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to open))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (handles ?current-obj ?handles)
    (desig-prop ?desig (angle ?angle))
    (available-arms ?current-obj ?available-arms)
    (optimal-handle-grasp ?current-obj ?available-arms ?grasp-assignments))

  (<- (action-desig ?desig (debug))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to debug)))
  
  (<- (action-desig ?desig (container-closed ?handle :right))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to close))
    (desig-prop ?desig (handle ?handle)))

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

(defun ik->trajectory (ik-result &key (duration 5.0) (stamp (ros-time)))
  (declare (type (or kinematics_msgs-srv:getpositionik-response
                     kinematics_msgs-srv:getconstraintawarepositionik-response)))
  "Converts the result of an IK call (type
arm_navigation_msgs/RobotState) to a joint trajectory message that can
be used in the corresponding actions."
  (roslisp:with-fields ((solution-names (name joint_state solution))
                        (solution-positions (position joint_state solution))
                        (error-code (val error_code)))
      ik-result
    (when (eql error-code 1)
      (roslisp:make-message
       "trajectory_msgs/JointTrajectory"
       (stamp header) stamp
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
  "Merges `trajectory' with `trajectories'. All trajectory point
velocities between the first point in `trajectory' and the last point
in the last trajectory in `trajectories' are set to `velocity'. This
method merges a number of trajectories, all for the same joints."
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
                                     velocities
                                     (map 'vector #'identity
                                          (make-list (length joint-names)
                                                     :initial-element velocity))
                                     accelerations
                                     (map 'vector #'identity
                                          (make-list (length joint-names)
                                                     :initial-element 0.0))
                                     time_from_start current-time)
                                    (roslisp:make-message
                                     "trajectory_msgs/JointTrajectoryPoint"
                                     positions positions
                                     velocities
                                     (map 'vector #'identity
                                          (make-list (length joint-names)
                                                     :initial-element 0.0))
                                     accelerations
                                     (map 'vector #'identity
                                          (make-list (length joint-names)
                                                     :initial-element 0.0))
                                     time_from_start current-time)))))
                        (concatenate 'vector points-1 points-2))))
             (cdr trajectories)))))
      trajectory))

(defun combine-trajectories (trajectory-1 trajectory-2
                             &key timestamp times-from-start)
  "Concatenates two trajectories over different joints, i.e. the
result is a joint trajectory that is the concatenation of the two
different (disjoint) joint sets.

The trajectories can only be combined if they have equal length, equal
headers and equal time_from_start slots. If `header' is set, the two
headers are not checked for equality and `header' is used instead. If
`times-from-start' is used, it must be a sequence of times that
replaces the time_from_start slots in the different trajectory
points."
  (declare (type trajectory_msgs-msg:jointtrajectory
                 trajectory-1 trajectory-2)
           (type (or null number) timestamp)
           (type (or null sequence) times-from-start))
  (roslisp:with-fields ((stamp-1 (stamp header))
                        (joint-names-1 joint_names)
                        (points-1 points))
      trajectory-1
    (roslisp:with-fields ((stamp-2 (stamp header))
                          (joint-names-2 joint_names)
                          (points-2 points))
        trajectory-2
      (let ((concatenated-joint-names (concatenate
                                       'vector joint-names-1 joint-names-2)))
        (unless (eql (length concatenated-joint-names)
                     (length (remove-duplicates concatenated-joint-names)))
          (error
           'simple-error
           :format-control "The joint name lists are not disjoint. ~a vs ~a."
           :format-arguments (list joint-names-1 joint-names-2)))
        (unless (eql (length points-1) (length points-2))
          (error
           'simple-error
           :format-control "The number of trajectory points differs."))
        (roslisp:make-msg
         "trajectory_msgs/JointTrajectory"
         (stamp header)
         (or
          timestamp
          (when (eql stamp-1 stamp-2)
            stamp-1)
          (error
           'simple-error
           :format-control
           "Cannot combine trajectories. Timestamps not equal. ~a != ~a"
           :format-arguments (list stamp-1 stamp-2)))
         joint_names (concatenate 'vector joint-names-1 joint-names-2)
         points
         (map 'vector (lambda (point-1 point-2 time-from-start)
                        (roslisp:with-fields ((positions-1 positions)
                                              (velocities-1 velocities)
                                              (accelerations-1 accelerations)
                                              (time-from-start-1
                                               time_from_start))
                            point-1
                          (roslisp:with-fields ((positions-2 positions)
                                                (velocities-2 velocities)
                                                (accelerations-2 accelerations)
                                                (time-from-start-2
                                                 time_from_start))
                              point-2
                            (roslisp:make-message
                             "trajectory_msgs/JointTrajectoryPoint"
                             positions (concatenate 'vector positions-1
                                                    positions-2)
                             velocities (concatenate 'vector velocities-1
                                                     velocities-2)
                             accelerations (concatenate 'vector accelerations-1
                                                        accelerations-2)
                             time_from_start
                             (or time-from-start
                                 (when (eql time-from-start-1 time-from-start-2)
                                   time-from-start-1)
                                 (error
                                  'simple-error
                                  :format-control "time_from_start slots don't match for points ~a and ~a."
                                  :format-arguments (list point-1 point-2)))))))
              points-1 points-2 (or times-from-start
                                    (make-list (length points-1) :initial-element nil))))))))

(defun calculate-tool-pose (grasp tool-length)
  (cl-transforms:transform->pose
   (cl-transforms:transform*
    (cl-transforms:make-transform
     (cl-transforms:make-3d-vector tool-length 0 0)
     (cl-transforms:make-quaternion 0 0 0 1))             
    (cl-transforms:transform-inv
     (cl-transforms:reference-transform grasp)))))

(defun tool-goal-pose->wrist-goal-pose (obj &key
           (tool (cl-transforms:make-pose
                  (cl-transforms:make-3d-vector 0 0 0)
                  (cl-transforms:make-quaternion 0 0 0 1))))
  "At which pose should be the wrist coordinate system so that the
tool coordinate system will be at the specified `tool' pose? "
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

(defun calc-seed-state-elbow-up (side &key (elbow-up t) (elbow-out t))
  ;; get names, current-values, lower-limits, and upper-limits for arm
  ;; build joint-state msg with current values for arm,
  ;; set first, second and third joint to values that
  ;; result in the desired elbow position, i.e. elbow up
  ;; and/or elobw out; then: return msg
  (assert (or (eql side :right) (eql side :left)) ()
          "Invalid side-parameter given: ~a" side)
  (multiple-value-bind (names current lower-limits upper-limits)
      (get-arm-state side)
    (let ((desired-positions
            (map 'vector #'identity
                 (loop for name across names collecting
                       ;; if a standard seed-state was requested
                       ;; fill the respective values (depending on side)
                       ;; else copy the current joint values
                       (cond 
                         ;; seed position for shoulder_pan to have arm out
                         ((and (equal (aref names 0) name)
                               elbow-out)
                          (cond ((eql side :right)
                                 (+ (/ PI 4) (gethash name lower-limits)))
                                ((eql side :left)
                                 (- (gethash name upper-limits) (/ PI 4)))
                                (t (gethash name current))))
                         ;; seed position for should_lift to have elbow up
                         ((and (equal (aref names 1) name)
                               elbow-up)
                          0.0)
                         ;; seed position for upper_arm_roll to have elbow up
                         ((and (equal (aref names 2) name)
                               elbow-up)
                          (cond ((eql side :right)
                                 (- (/ PI 2)))
                                ((eql side :left)
                                 (/ PI 2))
                                (t (gethash name current))))
                         ;; copy all the other joints in default
                         (t (gethash name current)))))))
      (roslisp:make-msg
       "sensor_msgs/JointState"
       (stamp header) 0
       name names
       position desired-positions
       velocity (make-array (length names)
                            :element-type 'float
                            :initial-element 0.0)
       effort (make-array (length names)
                          :element-type 'float
                          :initial-element 0.0)))))

(defun calc-goal-transform-for-picked-object (obj-desig goal-location-desig)
  "Calculates the goal pose of an object that has already been picked
up (`obj-desig') to get to `goal-location-desig' using its' location
designator, which is relative to one of the grippers. The result is a
transform in /base_footprint."
  ;; get location designator of object designator,
  ;; NOTE: this is relative to one of the grippers
  (let ((obj-loc (current-desig (desig-prop-value (current-desig obj-desig) 'at))))
    ;; make sure that object designator and its location designator
    ;; have a proper structure
    (assert obj-loc () "Object ~a needs to have an `at' property"
            obj-desig)
    (assert (or (eq (desig-prop-value obj-loc 'in) 'gripper)
                (eq (desig-prop-value obj-loc 'in) 'both-grippers)) ()
                "Object ~a needs to be in the gripper" obj-loc)
    (assert (desig-prop-value obj-loc 'pose) ()
            "Object ~a needs to have a `pose' property" obj-desig)
    (assert (desig-prop-value obj-loc 'z-offset) ()
            "Object ~a needs to have a `z-offset' property" obj-desig)
    ;; get z difference between grasping point and supporting plane of object
    (let* ((obj-pose-z-offset (desig-prop-value obj-loc 'z-offset))
           (location-in-robot (tf:transform-pose
                               *tf* :pose (reference (current-desig goal-location-desig))
                                    :target-frame "/base_footprint")))
      ;; calc transform of desired pose of object, adjusted by its z difference
      (cl-transforms:reference-transform
       (tf:copy-pose-stamped
        location-in-robot
        :origin (cl-transforms:v+
                 (cl-transforms:origin location-in-robot)
                 (cl-transforms:make-3d-vector 0 0 obj-pose-z-offset)))))))

(defun euclidean-distance (names-from positions-from names-to
                           positions-to arm &key target-links)
  "Calculates and returns the euclidean distance of a vector of links
`target-links' between two joint space configurations. The
configurations are given as vectors, each consisting of a `names' and
a `positions' vector. The `-from' and `-to' pairs are interchangable
here. If no target links are given, all available links will be used."
  (let ((pose-name-pairs-from (get-fk names-from positions-from arm
                                      :target-links target-links))
        (pose-name-pairs-to (get-fk names-to positions-to arm
                                    :target-links target-links))
        (distance-name-pairs nil))
    (loop for pose-name-pair-from in pose-name-pairs-from
          do (let ((var-val (assoc (car pose-name-pair-from)
                                   pose-name-pairs-to :test 'equal)))
               (when var-val
                 (push (cons
                        (car var-val)
                        (tf:v-dist
                         (tf:origin (cdr var-val))
                         (tf:origin (cdr pose-name-pair-from))))
                       distance-name-pairs))))
    distance-name-pairs))

(defun execute-torso-command (trajectory)
  (actionlib:call-goal
   *trajectory-action-torso*
   (actionlib:make-action-goal
       *trajectory-action-torso*
     :trajectory (remove-trajectory-joints
                  #("torso_lift_joint") trajectory :invert t))))

(defun move-spine (position)
  (let ((spine-lift-trajectory
          (roslisp:make-msg
           "trajectory_msgs/JointTrajectory"
           (stamp header) (ros-time)
           joint_names #("torso_lift_joint")
           points (vector
                   (roslisp:make-message
                    "trajectory_msgs/JointTrajectoryPoint"
                    positions (vector position)
                    velocities #(0)
                    accelerations #(0)
                    time_from_start 5.0)))))
    (ros-info (pr2 manip-pm)
                      "Moving spine to position ~a." position)
    (execute-torso-command spine-lift-trajectory)
    (ros-info (pr2 manip-pm) "Moving spine complete.")))
