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

(in-package :pp-plans)

;; NOTE: unfortunately, cpl:def-cram-function doesn't use the specified lambda list.
;; Because of that declare type statements do not work on variables from the lambda list,
;; and the auto completion of arguments is useless as well.
;; As we would really like to have declare statements, our plans are simple defuns.
;; If in the future one would want to use def-cram-function for plan transformations,
;; one can always def-cram-function that calls a normal function.
(defun pick-up (&key
                  ((:object ?object-designator))
                  ((:arm ?arm))
                  ((:gripper-opening ?gripper-opening))
                  ((:effort ?grip-effort))
                  ((:grasp ?grasp))
                  location-type
                  ((:left-reach-poses ?left-reach-poses))
                  ((:right-reach-poses ?right-reach-poses))
                  ((:left-grasp-poses ?left-grasp-poses))
                  ((:right-grasp-poses ?right-grasp-poses))
                  ((:left-lift-poses ?left-lift-poses))
                  ((:right-lift-poses ?right-lift-poses))
                &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type keyword ?arm ?grasp)
           (type number ?gripper-opening ?grip-effort)
           (type (or null list) ; yes, null is also list, but this is more readable
                 ?left-reach-poses ?right-reach-poses
                 ?left-grasp-poses ?right-grasp-poses
                 ?left-lift-poses ?right-lift-poses)
           (ignore location-type))
  "Open gripper, reach traj, grasp traj, close gripper, issue grasping event, lift."

  (cram-tf:visualize-marker (man-int:get-object-pose ?object-designator)
                            :r-g-b-list '(1 1 0) :id 300)

  (cpl:par
    (roslisp:ros-info (pick-place pick-up) "Opening gripper")
    (exe:perform
     (desig:an action
               (type setting-gripper)
               (gripper ?arm)
               (position ?gripper-opening)))
    (roslisp:ros-info (pick-place pick-up) "Reaching")
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           ;; (return)
           ))
      (exe:perform
       (desig:an action
                 (type reaching)
                 (object ?object-designator)
                 (left-poses ?left-reach-poses)
                 (right-poses ?right-reach-poses)))))
  (roslisp:ros-info (pick-place pick-up) "Grasping")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)
         ))
    (exe:perform
     (desig:an action
               (type grasping)
               (object ?object-designator)
               (left-poses ?left-grasp-poses)
               (right-poses ?right-grasp-poses))))
  (roslisp:ros-info (pick-place pick-up) "Gripping")
  (exe:perform
   (desig:an action
             (type gripping)
             (gripper ?arm)
             (effort ?grip-effort)
             (object ?object-designator)
             (grasp ?grasp)))
  (roslisp:ros-info (pick-place pick-up) "Lifting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type lifting)
               (left-poses ?left-lift-poses)
               (right-poses ?right-lift-poses))))
  (roslisp:ros-info (pick-place place) "Parking")
  (exe:perform
   (desig:an action
             (type parking-arms)
             ;; TODO: this will not work with dual-arm grasping
             ;; but as our ?arm is declared as a keyword,
             ;; for now this code is the right code
             (arms (?arm)))))



(defun place (&key
                ((:object ?object-designator))
                ((:target ?target-location-designator))
                ((:other-object ?other-object-designator))
                other-object-is-a-robot
                ((:arm ?arm))
                grasp
                location-type
                ((:gripper-opening ?gripper-opening))
                ((:attachment-type ?placing-location-name))
                ((:left-reach-poses ?left-reach-poses))
                ((:right-reach-poses ?right-reach-poses))
                ((:left-put-poses ?left-put-poses))
                ((:right-put-poses ?right-put-poses))
                ((:left-retract-poses ?left-retract-poses))
                ((:right-retract-poses ?right-retract-poses))
              &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type (or desig:object-designator null) ?other-object-designator)
           (type keyword ?arm)
           (type (or null keyword) ?placing-location-name)
           (type number ?gripper-opening)
           (type (or null list) ; yes, null is also list, but this is better readable
                 ?left-reach-poses ?right-reach-poses
                 ?left-put-poses ?right-put-poses
                 ?left-retract-poses ?right-retract-poses)
           (ignore grasp location-type))
  "Reach, put, assert assemblage if given, open gripper, retract grasp event, retract arm."

  (roslisp:ros-info (pick-place place) "Reaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         ;; (return)
         ))
    (exe:perform
     (desig:an action
               (type reaching)
               (location ?target-location-designator)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses))))
  (roslisp:ros-info (pick-place place) "Putting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type putting)
               (object ?object-designator)
               (desig:when ?other-object-designator
                 (supporting-object ?other-object-designator))
               (left-poses ?left-put-poses)
               (right-poses ?right-put-poses))))
  (when ?placing-location-name
    (roslisp:ros-info (boxy-plans connect) "Asserting assemblage connection in knowledge base")
    (if other-object-is-a-robot
        (cram-occasions-events:on-event
         (make-instance 'cpoe:object-attached-robot
           :link (roslisp-utilities:rosify-underscores-lisp-name
                  (desig:desig-prop-value ?other-object-designator :urdf-name))
           :not-loose t
           :object-name (desig:desig-prop-value ?object-designator :name)
           :other-object-name (desig:desig-prop-value ?other-object-designator :name)
           :grasp ?placing-location-name))
        (cram-occasions-events:on-event
         (make-instance 'cpoe:object-attached-object
           :object-name (desig:desig-prop-value ?object-designator :name)
           :other-object-name (desig:desig-prop-value ?other-object-designator :name)
           :attachment-type ?placing-location-name))))
  (roslisp:ros-info (pick-place place) "Opening gripper")
  (exe:perform
   (desig:an action
             (type setting-gripper)
             (gripper ?arm)
             (position ?gripper-opening)))
  (roslisp:ros-info (pick-place place) "Retract grasp in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
     :arm ?arm
     :object-name (desig:desig-prop-value ?object-designator :name)))
  (roslisp:ros-info (pick-place place) "Updating object location in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-location-changed
     :object-designator ?object-designator
     :location-designator ?target-location-designator))
  (roslisp:ros-info (pick-place place) "Retracting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type retracting)
               (left-poses ?left-retract-poses)
               (right-poses ?right-retract-poses))))
  (roslisp:ros-info (pick-place place) "Parking")
  (exe:perform
   (desig:an action
             (type parking-arms)
             (arms (?arm)))))


(defun perceive (&key
                   ((:object ?object-designator))
                   (object-chosing-function #'identity)
                 &allow-other-keys)
  (declare (type desig:object-designator ?object-designator))
  "Park arms and call DETECTING action."
  (exe:perform
   (desig:an action
             (type parking-arms)
             (not-neck T)))
  (exe:perform
   (desig:an action
             (type detecting)
             (object ?object-designator))))


#+the-stuff-below-with-action-phases-is-a-bit-awkward
(
 (defun perform-phases-in-sequence (action-designator)
   (declare (type desig:action-designator action-designator))
   (let ((phases (desig:desig-prop-value action-designator :phases)))
     (mapc (lambda (phase)
             (format t "Executing phase: ~%~a~%~%" phase)
             (exe:perform phase))
           phases)))

 (cpl:def-cram-function pick-up (action-designator object arm grasp)
   (perform-phases-in-sequence action-designator)
   (cram-occasions-events:on-event
    (make-instance 'cpoe:object-attached-robot :object object :arm arm :grasp grasp)))

 (cpl:def-cram-function place (action-designator object arm)
   (perform-phases-in-sequence action-designator)
   (cram-occasions-events:on-event
    (make-instance 'cpoe:object-detached-robot :arm arm :object object)))
 )
