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

(in-package :cram-btr-visibility-costmap)

(defun validate-designator-solution (desig pose)
  (cut:with-vars-bound (?checks)
      (cut:lazy-car (prolog `(btr-desig-solution-valid ,desig ,pose ?checks)))
    (cond ((cut:is-var ?checks) :reject)
          ((eq ?checks nil) :unknown)
          (t :accept))))

(desig:register-location-validation-function
 10 validate-designator-solution
 "Uses the bullet base reasoning system to validate a designator.")


#+everything-below-is-reachability-costmap-related-and-is-deprecated
(
 (defvar *designator-ik-check-cache*
   (tg:make-weak-hash-table :weakness :key)
   "Cache of closures to check IK for a specific designator.")

 (defvar *robot-valid-sides* '(:left :right))

 (defvar *check-ik-joint-states*
   '(("torso_lift_joint" 0.16825)
     ("torso_lift_joint" 0.33)
     ("torso_lift_joint" 0.0))
   "Joint states to set before calling the IK solver.")

 (register-location-validation-function
  2 check-ik-solution
  "For designators to reach, checks if IK solutions are present for all
 poses in ")
 (disable-location-validation-function 'check-ik-solution)
 Currently, this validation function is disabled as it takes way to long
 to "theoretically" check for IK results for a certain robot pose.

 (defun pose-side-properties (designator)
   "Returns a list of lists (pose side) that correspond to the poses
that `designator' tries to reach."
   (force-ll
    (lazy-mapcar
     (lambda (solution)
       (with-vars-bound (?pose ?side) solution
         (list ?pose (unless (is-var ?side) ?side))))
     (prolog `(cram-robot-interfaces:designator-reach-pose ,designator ?pose ?side)))))

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
   (cond ((not (cram-robot-interfaces:reachability-designator-p designator))
          :unknown)
         ((let ((cached-function
                  (or (gethash designator *designator-ik-check-cache*)
                      (setf (gethash designator *designator-ik-check-cache*)
                            (let* ((state (bullet:get-state *current-bullet-world*))
                                   (world (bullet:restore-world-state
                                           state (make-instance 'btr:bt-reasoning-world
                                                   :gravity-vector (bullet:gravity-vector state))))
                                   (robot (btr:object world (btr:get-robot-name))))
                              (make-ik-check-function
                               robot (pose-side-properties designator)))))))
            (funcall cached-function pose))
          :accept)))
)
