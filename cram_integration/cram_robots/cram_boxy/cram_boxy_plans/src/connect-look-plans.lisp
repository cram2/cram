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

(cpl:def-cram-function connect (?object-designator ?with-object-designator
                                ?arm ?gripper-opening attachment-type
                                ?left-reach-poses ?right-reach-poses
                                ?left-push-poses ?right-push-poses
                                ?left-retract-poses ?right-retract-poses)
  (roslisp:ros-info (boxy-plans connect) "Reaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (boxy-plans connect)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type reaching)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses))))
  (roslisp:ros-info (boxy-plans connect) "Pushing")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (boxy-plans connect)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type pushing ;; wiggling
                     )
               (left-poses ?left-push-poses)
               (right-poses ?right-push-poses))))
  (roslisp:ros-info (boxy-plans connect) "Asserting assemblage connection in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-attached-object
     :object-name (desig:desig-prop-value ?object-designator :name)
     :other-object-name (desig:desig-prop-value ?with-object-designator :name)
     :attachment-type attachment-type))
  (roslisp:ros-info (boxy-plans connect) "Opening gripper")
  (exe:perform
   (desig:an action
             (type setting-gripper)
             (gripper ?arm)
             (position ?gripper-opening)))
  (roslisp:ros-info (boxy-plans connect) "Retracting grasp in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
     :arm ?arm
     :object-name (desig:desig-prop-value ?object-designator :name)))
  (roslisp:ros-info (boxy-plans connect) "Retracting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (boxy-plans connect)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type retracting)
               (left-poses ?left-retract-poses)
               (right-poses ?right-retract-poses)))))


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


(defun move-arms-into-nicer-configuration ()
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e) ; ignore failures
         (roslisp:ros-warn (boxy-plans arm-nicer-config) "~a" e)
         (return)))
    (let ((?left-configuration boxy-descr::*left-arm-nicer-configuration*))
      (exe:perform
       (desig:a motion
                (type moving-arm-joints)
                (left-configuration ?left-configuration))))))
