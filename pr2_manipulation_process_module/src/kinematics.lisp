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

(defun get-gripper-state (side)
  "Returns the position of the gripper. 0 indicates a completely
closed gripper."
  (let ((joint-name (ecase side
                      (:right "r_gripper_joint")
                      (:left "l_gripper_joint"))))
    (cram-moveit:get-joint-value joint-name)))

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

(defun reaching-length (pose side
                        &key allowed-collision-objects highlight-links)
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
  (let* ((wrist-frame (cut:var-value
                       '?link
                       (first
                        (crs:prolog
                         `(manipulator-link ,side ?link)))))
         (arm-group (cut:var-value
                     '?group
                     (first
                      (crs:prolog
                       `(planning-group ,side ?group)))))
         (pose-in-tll
           (cl-tf2:transform-pose
            *tf2-buffer* :pose pose :target-frame  "/torso_lift_link"
                         :timeout *tf-default-timeout* :use-current-ros-time t)))
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
