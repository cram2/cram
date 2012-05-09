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

(in-package :btr-desig)

(defvar *designator-ik-check-cache*
  (tg:make-weak-hash-table :weakness :key)
  "Cache of closures to check IK for a specific designator.")

(defvar *robot-name* 'btr::pr2
  "The name of the robot in the bullet world to calculate IK for.")

(defvar *robot-valid-sides* '(:left :right))

(defvar *check-ik-joint-states*
  '(("torso_lift_joint" 0.33)
    ("torso_lift_joint" 0.0))
  "Joint states to set before calling the IK solver.")

(register-location-validation-function
 1 robot-location-on-floor
 "Verifies that the z coordinate of the robot is actually on the floor
 if searching for poses where the robot should stand")

(register-location-validation-function
 2 check-ik-solution
 "For designators to reach, checks if IK solutions are present for all
 poses in ")

(register-location-validation-function
 10 validate-designator-solution
 "Uses the bullet base reasoning system to validate a designator.")

(defmethod get-cost-map :around ((map location-costmap))
  (with-slots (cost-map) map
    (let ((initialized (slot-boundp map 'location-costmap::cost-map)))
      (prog1 (call-next-method)
        (unless initialized
          (add-costmap-function-object map))))))

(defun robot-location-on-floor (designator pose)
  (if (member (desig-prop-value designator 'to) '(reach see))
      (< (cl-transforms:z (cl-transforms:origin pose))
         0.05)
      t))

(defun reach-designator (designator)
  (prolog `(location-costmap:reachability-designator
            ,designator)))

(defun pose-side-properties (designator)
  "Returns a list of lists (pose side) that correspond to the poses
that `designator' tries to reach."
  (force-ll
   (lazy-mapcar
    (lambda (solution)
      (with-vars-bound (?pose ?side) solution
        (list ?pose (unless (is-var ?side) ?side))))
    (prolog `(designator-reach-pose ,designator ?pose ?side)))))

(defun check-reachability (robot reach-pose &key side)
  (etypecase reach-pose
    (cl-transforms:3d-vector (point-reachable-p robot reach-pose :side side))
    (cl-transforms:pose (pose-reachable-p robot reach-pose :side side))))

(defun make-ik-check-function (robot poses-and-sides)
  (flet ((check-reachability (reach-pose side)
           (case side
             ((:both :all)
              (every (lambda (side)
                       (check-reachability
                        robot reach-pose :side side))
                     *robot-valid-sides*))
             ((nil :either)
              (some (lambda (side)
                      (check-reachability
                       robot reach-pose :side side))
                    *robot-valid-sides*))
             (t (check-reachability
                 robot reach-pose :side side)))))
    (lambda (pose)
      (setf (btr:pose robot) pose)
      (every (lambda (pose-and-side-to-check)
               (if *check-ik-joint-states*
                   (some (lambda (joint-state)
                           (setf (btr:joint-state robot (car joint-state))
                                 (cadr joint-state))
                           (apply #'check-reachability pose-and-side-to-check))
                         *check-ik-joint-states*)
                   (apply #'check-reachability pose-and-side-to-check)))
             poses-and-sides))))

(defun check-ik-solution (designator pose)
  (or (not (reach-designator designator))
      (let ((cached-function
              (or (gethash designator *designator-ik-check-cache*)
                  (setf (gethash designator *designator-ik-check-cache*)
                        (let* ((state (bt:get-state *current-bullet-world*))
                               (world (bullet:restore-world-state
                                       state (make-instance 'btr:bt-reasoning-world
                                               :gravity-vector (bt:gravity-vector state))))
                               (robot (btr:object world *robot-name*)))
                          (make-ik-check-function
                           robot (pose-side-properties designator)))))))
        (funcall cached-function pose))))

(defun validate-designator-solution (desig pose)
  (prolog `(btr-desig-solution-valid ,desig ,pose)))
