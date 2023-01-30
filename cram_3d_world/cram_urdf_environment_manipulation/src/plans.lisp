;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :env-man)

(defparameter *detect-gripper-slip* T
  "When opening/closing a container, detect if gripper joint falls below
a threshold (if T) to signal a failure.")

(defun manipulate-container (&key
                               ((:type ?type))
                               ((:arm ?arm))
                               ((:gripper-opening ?gripper-opening))
                               ((:distance ?distance))
                               ((:absolute-distance ?absolute-distance))
                               ((:look-pose ?look-pose))
                               robot-arm-is-also-a-neck
                               ((:left-reach-poses ?left-reach-poses))
                               ((:right-reach-poses ?right-reach-poses))
                               ((:left-grasp-poses ?left-grasp-poses))
                               ((:right-grasp-poses ?right-grasp-poses))
                               ((:left-manipulate-poses ?left-manipulate-poses))
                               ((:right-manipulate-poses ?right-manipulate-poses))
                               ((:left-retract-poses ?left-retract-poses))
                               ((:right-retract-poses ?right-retract-poses))
                               joint-name
                               ((:link-name ?link-name))
                               ((:environment-name ?environment-name))
                               ((:environment-object ?environment-object))
                               ((:container-object ?container-designator))
                             &allow-other-keys)
  (declare (type keyword ?arm)
           (type number ?gripper-opening ?distance)
           (type list
                 ?left-reach-poses ?right-reach-poses
                 ?left-grasp-poses ?right-grasp-poses
                 ?left-manipulate-poses ?right-manipulate-poses
                 ?left-retract-poses ?right-retract-poses)
           (type (or string symbol null) joint-name ?link-name)
           (type (or symbol null) ?environment-name)
           (type desig:object-designator ?container-designator))

  ;;;;;;;;;;;;;;; OPEN GRIPPER AND REACH ;;;;;;;;;;;;;;;;
  (unless robot-arm-is-also-a-neck
    (roslisp:ros-info (env-manip plan) "Looking")
    (cpl:with-failure-handling
        ((common-fail:ptu-low-level-failure (e)
           (roslisp:ros-warn (env-manip plan)
                             "Looking-at had a problem: ~a~%Ignoring."
                             e)
           (return)))
      (exe:perform
       (desig:an action
                 (type looking)
                 (target (desig:a location
                                  (pose ?look-pose)))))))
  (cpl:par
    (roslisp:ros-info (env-manip plan) "Opening gripper and reaching")
    (let ((?goal `(cpoe:gripper-joint-at ,?arm ,?gripper-opening)))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?arm)
                 (position ?gripper-opening)
                 (goal ?goal))))
    (cpl:with-retry-counters ((reach-retries 2))
      (cpl:with-failure-handling
          ((common-fail:manipulation-low-level-failure (e)
             (roslisp:ros-warn (env-plans manipulate)
                               "Manipulation messed up: ~a~%Failing."
                               e)
             (cpl:do-retry reach-retries
               (cpl:retry))))
        (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
          (exe:perform
           (desig:an action
                     (type reaching)
                     (left-poses ?left-reach-poses)
                     (right-poses ?right-reach-poses)
                     (goal ?goal)))))))
  (cpl:with-retry-counters ((grasp-retries 2))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (env-plans manipulate)
                             "Manipulation messed up: ~a~%Failing."
                             e)
           (cpl:do-retry grasp-retries
             (cpl:retry))))
      (let ((?goal `(cpoe:tool-frames-at ,?left-grasp-poses ,?right-grasp-poses)))
        (exe:perform
         (desig:an action
                   (type grasping)
                   (object (desig:an object
                                     (name ?environment-name)))
                   (link ?link-name)
                   (left-poses ?left-grasp-poses)
                   (right-poses ?right-grasp-poses)
                   (goal ?goal))))))

  ;;;;;;;;;;;;;;;;;;;; GRIPPING ;;;;;;;;;;;;;;;;;;;;;;;;
  (roslisp:ros-info (environment-manipulation manipulate-container)
                    "Gripping")
  ;; Gripping now both for closing and opening, as grasp pose can be funny.
  ;; when (eq ?type :opening)
  (exe:perform
   (desig:an action
             (type gripping)
             (gripper ?arm)))

  ;;;;;;;;;;;;;;;;;;;;;; MANIPULATING ;;;;;;;;;;;;;;;;;;;;;;;
  (roslisp:ros-info (environment-manipulation manipulate-container)
                    "Manipulating")
  (let* ((?push-or-pull
           (if (eq ?type :opening)
               :pulling
               :pushing))
         (?goal
           `(cpoe:tool-frames-at ,?left-manipulate-poses ,?right-manipulate-poses))
         (manipulation-action
           (desig:an action
                     (type ?push-or-pull)
                     (object (desig:an object (name ?environment-name)))
                     (container-object ?container-designator)
                     (link ?link-name)
                     (desig:when ?absolute-distance
                       (distance ?absolute-distance))
                     (desig:when (eq ?arm :left)
                       (left-poses ?left-manipulate-poses))
                     (desig:when (eq ?arm :right)
                       (right-poses ?right-manipulate-poses))
                     (goal ?goal))))
    (if *detect-gripper-slip*
        ;; monitor gripper joint to detect if the handle slipped out of the gripper
        (cpl:pursue
         (cpl:with-failure-handling
             ((common-fail:manipulation-low-level-failure
               (e)
               (roslisp:ros-warn (env-plans manipulate)
                                 "Manipulation messed up: ~a~%Failing." e)))
           (exe:perform manipulation-action))
         (cpl:seq
          (exe:perform
           (desig:an action
                     (type monitoring-joint-state)
                     (gripper ?arm)))
          ;; sleep for two seconds,
          ;; maybe the action is nearly finished, so there is no need to fail
          (cpl:sleep 2)
          (cpl:fail 'common-fail:gripper-closed-completely
                    :description "Handle slipped")))
        ;; in unreal the gripper wiggles so much it falsely detects slipping
        (cpl:with-failure-handling
            ((common-fail:manipulation-low-level-failure
              (e)
              (roslisp:ros-warn (env-plans manipulate)
                                "Manipulation messed up: ~a~%Failing." e)))
          (exe:perform manipulation-action))))

  (when (and joint-name)
    (cram-occasions-events:on-event
     (make-instance (if (eq ?type :opening)
                        'cpoe:container-opening-event
                        'cpoe:container-closing-event)
       :joint-name joint-name
       :side ?arm
       :environment ?environment-object
       :distance ?distance)))

  ;;;;;;;;;;;;;;;;;;;; RETRACTING ;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (roslisp:ros-info (environment-manipulation manipulate-container)
                    "Retracting")
  (let ((?goal `(cpoe:gripper-opened ,?arm)))
    (exe:perform
     (desig:an action
               (type releasing)
               (gripper ?arm)
               (goal ?goal))))
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (env-plans manipulate)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (let ((?goal `(cpoe:tool-frames-at ,?left-retract-poses ,?right-retract-poses)))
      (exe:perform
       (desig:an action
                 (type retracting)
                 (left-poses ?left-retract-poses)
                 (right-poses ?right-retract-poses)
                 (goal ?goal)))))
  (exe:perform
   (desig:an action
             (type parking-arms))))
