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
      (roslisp:call-service
       (concatenate
        'string
        (ecase side
          (:right *ik-right-ns*)
          (:left *ik-left-ns*))
        "/get_ik_solver_info")
       "kinematics_msgs/GetKinematicSolverInfo")
    joint-names))

(defun make-seed-states (side joint-names &optional (steps 3))
  (flet ((init ()
           (let ((current (make-hash-table :test 'equal))
                 (lower (make-hash-table :test 'equal))
                 (upper (make-hash-table :test 'equal))
                 (joint-state (cpl:value *joint-state*)))
             (roslisp:with-fields ((joint-names (joint_names kinematic_solver_info))
                                   (limits (limits kinematic_solver_info)))
                 (roslisp:call-service
                  (concatenate
                   'string
                   (ecase side
                     (:right *ik-right-ns*)
                     (:left *ik-left-ns*))
                   "/get_ik_solver_info")
                  "kinematics_msgs/GetKinematicSolverInfo")
               (map nil (lambda (limit)
                          (roslisp:with-fields ((joint-name joint_name)
                                                (min min_position)
                                                (max max_position))
                              limit
                            (setf (gethash joint-name lower) min)
                            (setf (gethash joint-name upper) max)
                            (setf (gethash joint-name current)
                                  (get-joint-position joint-state joint-name))))
                    limits)
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
  (roslisp:with-fields ((solution-names (name solution))
                        (solution-positions (position solution))
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
                       (roslisp:with-fields (positions time_from_start)
                           point
                         (roslisp:make-message
                          "trajectory_msgs/JointTrajectoryPoint"
                          positions (map 'vector #'identity
                                         (loop for n across joint-names
                                               for p across positions
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
          (apply
           #'merge-trajectories
           velocity
           (roslisp:make-message
            "trajectory_msgs/JointTrajectory"
            (stamp header) stamp
            joint_names joint-names
            points (let ((time 0))
                     (map
                      'vector
                      (lambda (pt)
                        (roslisp:with-fields (positions time_from_start) pt
                          (prog1
                              (if (not (eq pt (elt points-2 (1- (length points-2)))))
                                  (roslisp:make-message
                                   "trajectory_msgs/JointTrajectoryPoint"
                                   positions positions
                                   velocities (map 'vector #'identity
                                                   (make-list (length joint-names)
                                                              :initial-element velocity))
                                   accelerations (map 'vector #'identity
                                                      (make-list (length joint-names)
                                                                 :initial-element 1.0))
                                   time_from_start (+ time time_from_start))
                                  (roslisp:make-message
                                   "trajectory_msgs/JointTrajectoryPoint"
                                   positions positions
                                   velocities (map 'vector #'identity
                                                   (make-list (length joint-names)
                                                              :initial-element 0.0))
                                   accelerations (map 'vector #'identity
                                                      (make-list (length joint-names)
                                                                 :initial-element 0.0))
                                   time_from_start (+ time time_from_start)))
                            (incf time time_from_start))))
                      (concatenate 'vector points-1 points-2))))
           (cdr trajectories))))
      trajectory))

(defun get-ik (side pose &key
               (tool (cl-transforms:make-pose
                      (cl-transforms:make-3d-vector 0 0 0)
                      (cl-transforms:make-quaternion 0 0 0 1)))
               (max-tries 30))
  (let ((seeds (make-seed-states
                side (remove "torso_lift_joint"
                             (get-joint-names side)
                             :test #'equal))))
    (lazy-list ((seeds (if max-tries
                           (lazy-take max-tries seeds)
                           seeds)))
      (when seeds
        (let ((result (roslisp:call-service
                       (concatenate
                        'string
                        (ecase side
                          (:right *ik-right-ns*)
                          (:left *ik-left-ns*))
                        "/get_weighted_ik")
                       'kdl_arm_kinematics-srv:getweightedik
                       :pose (tf:pose-stamped->msg pose)
                       :tool_frame (tf:pose->msg tool)
                       :ik_seed (lazy-car seeds))))
          (roslisp:with-fields ((error-code (val error_code)))
              result
            (if (eql error-code 1)
                (cont result (lazy-cdr seeds))
                (next (lazy-cdr seeds)))))))))

(defun get-sgp-grasps (side obj)
  (ecase side
    (:right (roslisp:set-param "/grasp_pcd/sgp_config_param_start_side" 0.0))
    (:left (roslisp:set-param "/grasp_pcd/sgp_config_param_start_side" pi)))
  (let ((grasps (roslisp:call-service
                 "/grasp_pcd/simple_grasp_planner" "sgp_srvs/sgp"
                 :query (jlo:partial-lo (perception-pm:object-jlo (reference obj))))))
    (roslisp:with-fields (poses quals) grasps
      (lazy-mapcar
       (lambda (g)
         (tf:msg->pose (car g)))
       (sort (map 'list #'cons poses quals)
             #'<
             :key #'cdr)))))

(defun get-grasp-object-trajectory-points (side obj)
  (flet ((calculate-tool-pose (grasp tool-length)
           (cl-transforms:transform->pose
            (cl-transforms:transform*
             (cl-transforms:make-transform
              (cl-transforms:make-3d-vector tool-length 0 0)
              (cl-transforms:make-quaternion 0 0 0 1))             
             (cl-transforms:transform-inv
              (cl-transforms:reference-transform grasp))))))
    (let ((grasps (get-sgp-grasps side obj))
          (pose (designator-pose obj)))
      (lazy-car
       (lazy-mapcan (lambda (grasp)
                      (format t "grasp: ~a~%" grasp)
                      (format t "tool: ~a~%" (calculate-tool-pose grasp 0.0))
                      (format t "tool: ~a~%" (calculate-tool-pose grasp 0.1))
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
                    grasps)))))

(defun get-grasp-object-trajectory (side obj)
  (flet ((calculate-tool-pose (grasp tool-length)
           (cl-transforms:transform->pose
            (cl-transforms:transform*
             (cl-transforms:make-transform
              (cl-transforms:make-3d-vector tool-length 0 0)
              (cl-transforms:make-quaternion 0 0 0 1))             
             (cl-transforms:transform-inv
              (cl-transforms:reference-transform grasp))))))
    (let ((grasps (get-sgp-grasps side obj))
          (pose (designator-pose obj)))
      (lazy-car
       (lazy-mapcan (lambda (grasp)
                      (format t "grasp: ~a~%" grasp)
                      (format t "tool: ~a~%" (calculate-tool-pose grasp 0.0))
                      (format t "tool: ~a~%" (calculate-tool-pose grasp 0.1))
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
                    grasps)))))
