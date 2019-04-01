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

(defun pose->transform-stamped-in-base (pose child-frame-lispy)
  (let ((target-pose-in-base
          (cram-tf:ensure-pose-in-frame
           pose cram-tf:*robot-base-frame* :use-zero-time t))
        (child-frame-rosy
          (roslisp-utilities:rosify-underscores-lisp-name child-frame-lispy)))
    (cram-tf:pose-stamped->transform-stamped target-pose-in-base child-frame-rosy)))


(def-fact-group pick-and-place-plans (desig:action-grounding)

  (<- (desig:action-grounding ?action-designator (pick-up ?resolved-action-designator))
    (spec:property ?action-designator (:type :picking-up))

    ;; extract info from ?action-designator
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    (spec:property ?current-object-desig (:name ?object-name))
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (man-int:robot-free-hand ?_ ?arm))
    (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)

    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
    (-> (man-int:object-rotationally-symmetric ?object-type)
        (equal ?rotationally-symmetric t)
        (equal ?rotationally-symmetric nil))
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (and (lisp-fun man-int:get-object-type-grasps ?object-type ?arm ?object-transform ?grasps)
             (member ?grasp ?grasps)))
    (lisp-fun man-int:get-object-type-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-object-type-gripper-opening ?object-type ?gripper-opening)

    ;; calculate trajectory
    (equal ?objects (?current-object-desig))
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory :picking-up ?arm ?grasp ?objects
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :grasping
                       ?left-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :lifting
                       ?left-lift-poses))
        (and (equal ?left-reach-poses NIL)
             (equal ?left-grasp-poses NIL)
             (equal ?left-lift-poses NIL)))
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory :picking-up ?arm ?grasp ?objects
                       ?right-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
                       ?right-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grasping
                       ?right-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :lifting
                       ?right-lift-poses))
        (and (equal ?right-reach-poses NIL)
             (equal ?right-grasp-poses NIL)
             (equal ?right-lift-poses NIL)))
    (or (lisp-pred identity ?left-trajectory)
        (lisp-pred identity ?right-trajectory))

    ;; put together resulting action designator
    (desig:designator :action ((:type :picking-up)
                               (:object ?current-object-desig)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:effort ?effort)
                               (:grasp ?grasp)
                               (:left-reach-poses ?left-reach-poses)
                               (:right-reach-poses ?right-reach-poses)
                               (:left-grasp-poses ?left-grasp-poses)
                               (:right-grasp-poses ?right-grasp-poses)
                               (:left-lift-poses ?left-lift-poses)
                               (:right-lift-poses ?right-lift-poses))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (place ?resolved-action-designator))
    (spec:property ?action-designator (:type :placing))

    ;; find in which hand the object is
    (-> (spec:property ?action-designator (:arm ?arm))
        (-> (spec:property ?action-designator (:object ?object-designator))
            (or (cpoe:object-in-hand ?object-designator ?arm)
                (format "WARNING: Wanted to place an object ~a with arm ~a, ~
                         but it's not in the arm.~%" ?object-designator ?arm))
            (cpoe:object-in-hand ?object-designator ?arm))
        (-> (spec:property ?action-designator (:object ?object-designator))
            (or (cpoe:object-in-hand ?object-designator ?arm)
                (format "WARNING: Wanted to place an object ~a ~
                         but it's not in any of the hands.~%" ?object-designator))
            (cpoe:object-in-hand ?object-designator ?arm)))

    ;;; infer missing information
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:type ?object-type))
    (spec:property ?current-object-designator (:name ?object-name))
    (lisp-fun man-int:get-object-type-gripper-opening ?object-type ?gripper-opening)

    ;; take object-pose from action-designator :target otherwise from object-designator pose
    (-> (spec:property ?action-designator (:target ?location-designator))
        (and (desig:current-designator ?location-designator ?current-location-designator)
             (desig:designator-groundings ?current-location-designator ?poses)
             (member ?target-object-pose ?poses)
             (lisp-fun pose->transform-stamped-in-base ?target-object-pose ?object-name
                       ?target-object-transform))
        (and (lisp-fun man-int:get-object-transform ?current-object-designator
                       ?target-object-transform)
             (lisp-fun man-int:get-object-pose ?current-object-designator ?target-object-pose)
             (desig:designator :location ((:pose ?target-object-pose))
                               ?current-location-designator)))

    ;; placing happens on/in an object
    (or (desig:desig-prop ?current-location-designator (:on ?other-object-designator))
        (desig:desig-prop ?current-location-designator (:in ?other-object-designator))
        (equal ?other-object-designator NIL))
    (-> (desig:desig-prop ?current-location-designator (:attachment ?placement-location-name))
        (true)
        (equal ?placement-location-name NIL))

    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (cpoe:object-in-hand ?object-designator ?arm ?grasp))

    ;; calculate trajectory
    (equal ?objects (?current-object-designator))
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory
                       :placing ?arm ?grasp ?objects
                       :target-object-transform-in-base ?target-object-transform
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :putting
                       ?left-put-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retracting
                       ?left-retract-poses))
        (and (equal ?left-reach-poses NIL)
             (equal ?left-put-poses NIL)
             (equal ?left-retract-poses NIL)))
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory
                       :placing ?arm ?grasp ?objects
                       :target-object-transform-in-base ?target-object-transform
                       ?right-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
                       ?right-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :putting
                       ?right-put-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :retracting
                       ?right-retract-poses))
        (and (equal ?right-reach-poses NIL)
             (equal ?right-put-poses NIL)
             (equal ?right-retract-poses NIL)))
    (or (lisp-pred identity ?left-trajectory)
        (lisp-pred identity ?right-trajectory))

    ;; put together resulting designator
    (desig:designator :action ((:type :placing)
                               (:object ?current-object-designator)
                               (:other-object ?other-object-designator)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:target ?current-location-designator)
                               (:placement-location-name ?placement-location-name)
                               (:left-reach-poses ?left-reach-poses)
                               (:right-reach-poses ?right-reach-poses)
                               (:left-put-poses ?left-put-poses)
                               (:right-put-poses ?right-put-poses)
                               (:left-retract-poses ?left-retract-poses)
                               (:right-retract-poses ?right-retract-poses))
                      ?resolved-action-designator)))
