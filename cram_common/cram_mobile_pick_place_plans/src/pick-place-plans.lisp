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
;; Actually, the main thing we use in the task tree is the PERFORM calls,
;; which are already stored in the task tree. So there is no need for
;; def-cram-function as of now anymore.
(defun pick-up (&key
                  ((:object ?object-designator))
                  ((:arm ?arm))
                  ((:gripper-opening ?gripper-opening))
                  ((:effort ?grip-effort))
                  ((:left-grasp ?left-grasp))
                  ((:right-grasp ?right-grasp))
                  location-type
                  ((:look-pose ?look-pose))
                  ((:left-reach-poses ?left-reach-poses))
                  ((:right-reach-poses ?right-reach-poses))
                  ((:left-grasp-poses ?left-grasp-poses))
                  ((:right-grasp-poses ?right-grasp-poses))
                  ((:left-lift-poses ?left-lift-poses))
                  ((:right-lift-poses ?right-lift-poses))
                  ((:hold ?holding))
                &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type list ?arm)
           ;;(type (or nil keyword) ?left-grasp ?right-grasp)
           (type number ?gripper-opening ?grip-effort)
           (type (or null list) ; yes, null is also a list, but this is more readable
                 ?left-reach-poses ?right-reach-poses
                 ?left-grasp-poses ?right-grasp-poses
                 ?left-lift-poses ?right-lift-poses)
           (ignore location-type))
  "Open gripper, reach traj, grasp traj, close gripper, issue grasping event, lift."

  (cram-tf:visualize-marker (man-int:get-object-pose ?object-designator)
                            :r-g-b-list '(1 1 0) :id 300)

  (roslisp:ros-info (pick-place pick-up) "Looking")
  (cpl:with-failure-handling
      ((common-fail:ptu-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Looking-at had a problem: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type looking)
               (target (desig:a location
                                (pose ?look-pose))))))
  (cpl:par
    (roslisp:ros-info (pick-place pick-up) "Opening gripper and reaching")
    (let ((?goal `(ccoe:gripper-joint-at ,?arm ,?gripper-opening)))
      (exe:perform
       (desig:an action
                 (type setting-gripper)
                 (gripper ?arm)
                 (position ?gripper-opening)
                 (goal ?goal))))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (let ((?goal `(ccoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
        (exe:perform
         (desig:an action
                   (type reaching)
                   (object ?object-designator)
                   (left-poses ?left-reach-poses)
                   (right-poses ?right-reach-poses)
                   (goal ?goal))))))
  (roslisp:ros-info (pick-place pick-up) "Grasping")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (let ((?goal `(ccoe:tool-frames-at ,?left-grasp-poses ,?right-grasp-poses)))
      (exe:perform
       (desig:an action
                 (type grasping)
                 (object ?object-designator)
                 (left-poses ?left-grasp-poses)
                 (right-poses ?right-grasp-poses)
                 (goal ?goal)))))
  (let ((?goal `(ccoe:object-in-hand ,?object-designator ,?arm)))
    (roslisp:ros-info (pick-place pick-up) "Gripping")
    (when (member :left ?arm)
      (exe:perform
       (desig:an action
                 (type gripping)
                 (gripper :left)
                 (effort ?grip-effort)
                 (object ?object-designator)
                 (desig:when ?left-grasp
                   (grasp ?left-grasp))
                 (goal ?goal))))
    (when (member :right ?arm)
      (exe:perform
       (desig:an action
                 (type gripping)
                 (gripper :right)
                 (effort ?grip-effort)
                 (object ?object-designator)
                 (desig:when ?right-grasp
                   (grasp ?right-grasp))
                 (goal ?goal)))))
  (unless ?holding
   (roslisp:ros-info (pick-place pick-up) "Lifting")
   (cpl:pursue
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (let ((?goal `(ccoe:tool-frames-at ,?left-lift-poses ,?right-lift-poses)))
        (exe:perform
         (desig:an action
                   (type lifting)
                   (left-poses ?left-lift-poses)
                   (right-poses ?right-lift-poses)
                   (goal ?goal)))))
    ;; (cpl:seq
    ;;   (exe:perform
    ;;    (desig:an action
    ;;              (type monitoring-joint-state)
    ;;              (gripper ?arm)))
    ;;   (cpl:fail 'common-fail:gripper-closed-completely
    ;;             :description "Object slipped"))
     )
  (roslisp:ros-info (pick-place place) "Parking")
  (exe:perform
   (desig:an action
             (type parking-arms)
             ;; TODO: this will not work with dual-arm grasping
             ;; but as our ?arm is declared as a keyword,
             ;; for now this code is the right code
             (arms (?arm))))))

(defun place (&key
                ((:object ?object-designator))
                ((:target ?target-location-designator))
                ((:other-object ?other-object-designator))
                other-object-is-a-robot
                ((:arm ?arm))
                ((:left-grasp ?left-grasp))
                ((:right-grasp ?right-grasp))
                location-type
                ((:gripper-opening ?gripper-opening))
                ((:attachment-type ?placing-location-name))
                ((:look-pose ?look-pose))
                ((:left-reach-poses ?left-reach-poses))
                ((:right-reach-poses ?right-reach-poses))
                ((:left-put-poses ?left-put-poses))
                ((:right-put-poses ?right-put-poses))
                ((:left-retract-poses ?left-retract-poses))
                ((:right-retract-poses ?right-retract-poses))
              &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type (or desig:object-designator null) ?other-object-designator)
           (type list ?arm)
           (type (or null keyword) ?placing-location-name ?left-grasp ?right-grasp)
           (type number ?gripper-opening)
           (type (or null list) ; yes, null is also list, but this is better readable
                 ?left-reach-poses ?right-reach-poses
                 ?left-put-poses ?right-put-poses
                 ?left-retract-poses ?right-retract-poses)
           (ignore location-type))
  "Reach, put, assert assemblage if given, open gripper, retract grasp event, retract arm."

  (roslisp:ros-info (pick-place place) "Looking")
  (cpl:with-failure-handling
      ((common-fail:ptu-low-level-failure (e)
         (roslisp:ros-warn (pp-plans place)
                           "Looking-at had a problem: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type looking)
               (target (desig:a location
                                (pose ?look-pose))))))
  (roslisp:ros-info (pick-place place) "Reaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         ;; (return)
         ))
    (let ((?goal `(ccoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
      (exe:perform
       (desig:an action
                 (type reaching)
                 (location ?target-location-designator)
                 (left-poses ?left-reach-poses)
                 (right-poses ?right-reach-poses)
                 (goal ?goal)))))
  (roslisp:ros-info (pick-place place) "Putting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (let ((?goal `(ccoe:tool-frames-at ,?left-put-poses ,?right-put-poses)))
      (exe:perform
       (desig:an action
                 (type putting)
                 (object ?object-designator)
                 (desig:when ?other-object-designator
                   (supporting-object ?other-object-designator))
                 (left-poses ?left-put-poses)
                 (right-poses ?right-put-poses)
                 (goal ?goal)))))
  (when ?placing-location-name
    (roslisp:ros-info (boxy-plans connect) "Asserting assemblage connection in knowledge base")
    (if other-object-is-a-robot
        (cram-occasions-events:on-event
         (make-instance 'ccoe:object-attached-robot
           :link (roslisp-utilities:rosify-underscores-lisp-name
                  (desig:desig-prop-value ?other-object-designator :urdf-name))
           :not-loose t
           :object-name (desig:desig-prop-value ?object-designator :name)
           :other-object-name (desig:desig-prop-value ?other-object-designator :name)
           :grasp ?placing-location-name))
        (cram-occasions-events:on-event
         (make-instance 'ccoe:object-attached-object
           :object-name (desig:desig-prop-value ?object-designator :name)
           :other-object-name (desig:desig-prop-value ?other-object-designator :name)
           :attachment-type ?placing-location-name))))
  (roslisp:ros-info (pick-place place) "Opening gripper")
  (let ((?goal `(ccoe:gripper-joint-at ,?arm ,?gripper-opening)))
    (exe:perform
     (desig:an action
               (type setting-gripper)
               (gripper ?arm)
               (position ?gripper-opening)
               (goal ?goal))))
  (roslisp:ros-info (pick-place place) "Retract grasp in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'ccoe:object-detached-robot
                  :arm ?arm
                  :object-name (desig:desig-prop-value ?object-designator :name)))
  (roslisp:ros-info (pick-place place) "Updating object location in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'ccoe:object-location-changed
     :object-designator ?object-designator
     :location-designator ?target-location-designator))
  (roslisp:ros-info (pick-place place) "Retracting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (let ((?goal `(ccoe:tool-frames-at ,?left-retract-poses ,?right-retract-poses)))
      (exe:perform
       (desig:an action
                 (type retracting)
                 (left-poses ?left-retract-poses)
                 (right-poses ?right-retract-poses)
                 (goal ?goal)))))
  (roslisp:ros-info (pick-place place) "Parking")
  (exe:perform
   (desig:an action
             (type parking-arms)
             (arms (?arm)))))


(defun park-arms (&key
                    ((:left-arm ?left-arm-p))
                    ((:right-arm ?right-arm-p))
                  &allow-other-keys)
  (declare (type boolean ?left-arm-p ?right-arm-p))
  "Puts the arms into a parking configuration"
  (let* ((left-config (when ?left-arm-p :park))
         (right-config (when ?right-arm-p :park))
         (?goal `(ccoe:arms-positioned-at ,left-config ,right-config)))
    (exe:perform
     (desig:an action
               (type positioning-arm)
               (desig:when ?left-arm-p
                 (left-configuration park))
               (desig:when ?right-arm-p
                 (right-configuration park))
               (goal ?goal)))))


(defun perceive (&key
                   ((:object ?object-designator))
                   park-arms
                   ((:counter ?counter))
                   ((:occluding-names ?occluding))
                   (object-chosing-function #'identity)
                 &allow-other-keys)
  (declare (type desig:object-designator ?object-designator)
           (type boolean park-arms)
           (ignore object-chosing-function))
  "Park arms and call DETECTING action."
  (when park-arms
    (exe:perform
     (desig:an action
               (type parking-arms)
               (not-neck T))))
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
    (make-instance 'ccoe:object-attached-robot :object object :arm arm :grasp grasp)))

 (cpl:def-cram-function place (action-designator object arm)
   (perform-phases-in-sequence action-designator)
   (cram-occasions-events:on-event
    (make-instance 'ccoe:object-detached-robot :arm arm :object object)))
 )
