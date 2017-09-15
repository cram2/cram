;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :boxy-plans)

(defun perform-phases-in-sequence (action-designator)
  (declare (type desig:action-designator action-designator))
  (let ((phases (desig:desig-prop-value action-designator :phases)))
    (mapc (lambda (phase)
            (format t "Executing phase: ~%~a~%~%" phase)
            (exe:perform phase))
          phases)))


(cpl:def-cram-function pick-up (?arm object-designator grasp
                                ?gripper-opening  ?grip-effort
                                ?left-reach-poses ?right-reach-poses
                                ?left-lift-poses ?right-lift-poses)
  (cpl:par
    (roslisp:ros-info (boxy-plans pick-up) "Opening gripper")
    (exe:perform
     (desig:an action
               (type setting-gripper)
               (gripper ?arm)
               (position ?gripper-opening)))
    (roslisp:ros-info (boxy-plans pick-up) "Reaching")
    (exe:perform
     (desig:an action
               (type reaching)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses))))
  (roslisp:ros-info (boxy-plans pick-up) "Gripping")
  (exe:perform
   (desig:an action
             (type gripping)
             (arm ?arm)
             (effort ?grip-effort)))
  (roslisp:ros-info (boxy-plans pick-up) "Assert grasp into knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-gripped :object object-designator :arm ?arm :grasp grasp))
  (roslisp:ros-info (boxy-plans pick-up) "Lifting")
  (exe:perform
   (desig:an action
             (type lifting)
             (left-poses ?left-lift-poses)
             (right-poses ?right-lift-poses))))


(cpl:def-cram-function place (?arm object-designator
                              ?left-reach-poses ?right-reach-poses
                              ?left-put-poses ?right-put-poses
                              ?left-retract-poses ?right-retract-poses)
  (roslisp:ros-info (boxy-plans place) "Reaching")
  (exe:perform
   (desig:an action
             (type reaching)
             (left-poses ?left-reach-poses)
             (right-poses ?right-reach-poses)))
  (roslisp:ros-info (boxy-plans place) "Putting")
  (exe:perform
   (desig:an action
             (type putting)
             (left-poses ?left-put-poses)
             (right-poses ?right-put-poses)))
  (roslisp:ros-info (boxy-plans place) "Opening gripper")
  (exe:perform
   (desig:an action
             (type releasing)
             (gripper ?arm)))
  (roslisp:ros-info (boxy-plans place) "Retract grasp in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-released :arm ?arm :object object-designator))
  (roslisp:ros-info (boxy-plans place) "Retracting")
  (exe:perform
   (desig:an action
             (type retracting)
             (left-poses ?left-retract-poses)
             (right-poses ?right-retract-poses))))

(cpl:def-cram-function connect (?arm object-designator with-object-designator
                                ?gripper-opening
                                ?left-reach-poses ?right-reach-poses
                                ?left-push-poses ?right-push-poses
                                ?left-retract-poses ?right-retract-poses)
  (roslisp:ros-info (boxy-plans connect) "Reaching")
  (exe:perform
   (desig:an action
             (type reaching)
             (left-poses ?left-reach-poses)
             (right-poses ?right-reach-poses)))
    (roslisp:ros-info (boxy-plans place) "Pushing")
  (exe:perform
   (desig:an action
             (type pushing)
             (left-poses ?left-push-poses)
             (right-poses ?right-push-poses)))
  (roslisp:ros-info (boxy-plans place) "Asserting assemblage connection in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'kr-assembly::object-attached
     :object object-designator
     :with-object with-object-designator))
  (roslisp:ros-info (boxy-plans place) "Opening gripper")
  (exe:perform
   (desig:an action
             (type setting-gripper)
             (gripper ?arm)
             (position ?gripper-opening)))
  (roslisp:ros-info (boxy-plans place) "Retracting grasp in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-released :arm ?arm :object object-designator))
  (roslisp:ros-info (boxy-plans place) "Retracting")
  (exe:perform
   (desig:an action
             (type retracting)
             (left-poses ?left-retract-poses)
             (right-poses ?right-retract-poses))))


(cpl:def-cram-function look (?left-goal-pose ?right-goal-pose)
  (roslisp:ros-info (boxy-plans look) "Looking with wrist camera")
  (exe:perform (desig:an action
                         (type reaching)
                         (left-poses (?left-goal-pose))
                         (right-poses (?right-goal-pose))))
  (cpl:sleep 1.0)
  (print "slept 1")
  (cpl:sleep 1.0)
  (print "slept 2"))


(defun move-arms-from-field-of-view ()
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e) ; ignore failures
         (roslisp:ros-warn (boxy-plans arm-from-field-of-view) "~a" e)
         (return)))
    (let ((?left-configuration kr-assembly::*left-arm-out-of-field-of-view-state*))
      (exe:perform
       (desig:a motion
                (type moving-arm-joints)
                (left-configuration ?left-configuration))))))

(defun move-arms-into-nicer-configuration ()
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e) ; ignore failures
         (roslisp:ros-warn (boxy-plans arm-nicer-config) "~a" e)
         (return)))
    (let ((?left-configuration kr-assembly::*left-arm-nicer-configuration*))
      (exe:perform
       (desig:a motion
                (type moving-arm-joints)
                (left-configuration ?left-configuration))))))

(defun move-neck-closer-to-look ()
  (let ((?configuration kr-assembly::*neck-good-looking-state*))
    (exe:perform
     (desig:a motion
              (type looking)
              (configuration ?configuration)))))

(defun move-neck-out-of-arm-workspace ()
  (let ((?configuration kr-assembly::*neck-out-of-arm-workspace-state*))
    (exe:perform
     (desig:a motion
              (type looking)
              (configuration ?configuration)))))


(cpl:def-cram-function detect (?object-designator)
  (let ((result-designator
          (cpl:with-retry-counters ((perceive-retries 5))
            (cpl:with-failure-handling
                ((common-fail:perception-object-not-found (e)
                   (cpl:do-retry perceive-retries
                     (roslisp:ros-warn (boxy-plans detect) "~a" e)
                     (cpl:retry))))
              (exe:perform
               (desig:a motion
                        (type detecting)
                        (object ?object-designator)))
              ;; (cram-robosherlock:perceive :detect object-designator)
              ))))

    result-designator))
