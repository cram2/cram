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

(defun manipulate-container (&key
                               ((:type ?type))
                               ((:arm ?arm))
                               ((:gripper-opening ?gripper-opening))
                               distance
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
           (type number ?gripper-opening distance)
           (type list
                 ?left-reach-poses ?right-reach-poses
                 ?left-grasp-poses ?right-grasp-poses
                 ?left-manipulate-poses ?right-manipulate-poses
                 ?left-retract-poses ?right-retract-poses)
           (type (or string symbol null) joint-name ?link-name)
           (type (or symbol null) ?environment-name)
           (type desig:object-designator ?container-designator))

  ;;;;;;;;;;;;;;; OPEN GRIPPER AND REACH ;;;;;;;;;;;;;;;;
  (cpl:par
    (roslisp:ros-info (environment-manipulation manipulate-container)
                      "Opening gripper")
    (exe:perform
     (desig:an action
               (type setting-gripper)
               (gripper ?arm)
               (position ?gripper-opening)))
    (roslisp:ros-info (environment-manipulation manipulate-container)
                      "Reaching")
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (env-plans manipulate)
                             "Manipulation messed up: ~a~%Failing."
                             e)
           ;; (return)
           ))
      (let ((?goal `(cpoe:ees-at ?left-reach-poses ?right-reach-poses)))
        (exe:perform
         (desig:an action
                   (type reaching)
                   (left-poses ?left-reach-poses)
                   (right-poses ?right-reach-poses)
                   (goal ?goal))))))

  ;;;;;;;;;;;;;;;;;;;; GRIPPING ;;;;;;;;;;;;;;;;;;;;;;;;
  (roslisp:ros-info (environment-manipulation manipulate-container)
                    "Gripping")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (env-plans manipulate)
                           "Manipulation messed up: ~a~%Failing."
                           e)
         ;; (return)
         ))
    (let ((?goal `(cpoe:ees-at left-grasp-poses ?right-grasp-poses)))
      (exe:perform
       (desig:an action
                 (type grasping)
                 (object (desig:an object
                                   (name ?environment-name)))
                 (link ?link-name)
                 (left-poses ?left-grasp-poses)
                 (right-poses ?right-grasp-poses)
                 (goal ?goal)))))
  (when (eq ?type :opening)
    (exe:perform
     (desig:an action
               (type gripping)
               (gripper ?arm))))

  ;;;;;;;;;;;;;;;;;;;;;; MANIPULATING ;;;;;;;;;;;;;;;;;;;;;;;
  (roslisp:ros-info (environment-manipulation manipulate-container)
                    "Manipulating")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (env-plans manipulate)
                           "Manipulation messed up: ~a~%Failing."
                           e)
         ;; (return)
         ))
    (let ((?push-or-pull (if (eq ?type :opening)
                            :pulling
                            :pushing))
          (?goal `(cpoe:ees-at ?left-manipulate-poses ?right-manipulate-poses)))
      ;; TODO(cpo) Have goal for opened container?
      (exe:perform
       (desig:an action
                 (type ?push-or-pull)
                 (object (desig:an object
                                   (name ?environment-name)))
                 (container-object ?container-designator)
                 (link ?link-name)
                 (left-poses ?left-manipulate-poses)
                 (right-poses ?right-manipulate-poses)
                 (goal ?goal)))))

  (when (and joint-name)
    (cram-occasions-events:on-event
     (make-instance (if (eq ?type :opening)
                        'cpoe:container-opening-event
                        'cpoe:container-closing-event)
       :joint-name joint-name
       :side ?arm
       :environment ?environment-object
       :distance distance)))

  ;;;;;;;;;;;;;;;;;;;; RETRACTING ;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (roslisp:ros-info (environment-manipulation manipulate-container)
                    "Retracting")
  (exe:perform
   (desig:an action
             (type releasing)
             (gripper ?arm)))
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (env-plans manipulate)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (let ((?goal `(cpoe:ees-at ?left-retract-poses ?right-retract-poses)))
      (exe:perform
       (desig:an action
                 (type retracting)
                 (left-poses ?left-retract-poses)
                 (right-poses ?right-retract-poses)
                 (goal ?goal))))))
