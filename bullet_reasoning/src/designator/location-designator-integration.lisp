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

(defvar *designator-reasoning-world-cache*
  (tg:make-weak-hash-table :weakness :key)
  "Cache of reasoning worlds used for ik calculation during designator
  validation.")

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

(defun check-ik-solution (designator pose)
  (flet ((make-ik-check-function (robot &optional side)
           (case side
             ((:left :right)
              (lambda (reach-pose)
                (pose-reachable-p
                 robot reach-pose
                 :side side :grasp :front)))
             (:both
              (lambda (reach-pose)
                (and
                 (pose-reachable-p
                  robot reach-pose
                  :side :right :grasp :front)
                 (pose-reachable-p
                  robot reach-pose
                  :side :left :grasp :front))))
             ((nil :either)
              (lambda (reach-pose)
                (or
                 (pose-reachable-p
                  robot reach-pose
                  :side :right :grasp :front)
                 (pose-reachable-p
                  robot reach-pose
                  :side :left :grasp :front)))))))
    (if (eq (desig-prop-value designator 'to) 'reach)
        ;; TODO(moesenle): add support for obj, object and location.
        (let* ((poses (append (desig-prop-values designator 'pose)))
               (side (desig-prop-value designator 'side))
               (world (or (gethash designator *designator-reasoning-world-cache*)
                          (setf (gethash designator *designator-reasoning-world-cache*)
                                (let ((state (bt:get-state *current-bullet-world*)))
                                  (bullet:restore-world-state
                                   state
                                   (make-instance 'btr:bt-reasoning-world
                                     :gravity-vector (bt:gravity-vector state)))))))
               ;; TODO(moesenle): don't hard-code robot name.
               (robot (btr:object world 'btr::pr2)))
          (setf (btr:pose robot) pose)
          (every (make-ik-check-function robot side) poses))
        t)))

(defun validate-designator-solution (desig pose)
  (prolog `(btr-desig-solution-valid ,desig ,pose)))
