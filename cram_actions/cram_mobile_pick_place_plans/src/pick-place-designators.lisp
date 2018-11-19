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

(defun extract-pick-up-manipulation-poses (arm left-manipulation-poses right-manipulation-poses)
  "`?arm' can be :left, :right or (:left :right)."
  (let ((arm-as-list (if (listp arm) arm (list arm)))
        left-reach-poses left-lift-poses
        left-grasp-poses right-grasp-poses
        right-reach-poses right-lift-poses)
    (when (member :left arm-as-list)
      (setf left-reach-poses (subseq left-manipulation-poses 0 2)
            left-grasp-poses (subseq left-manipulation-poses 2 3)
            left-lift-poses (subseq left-manipulation-poses 3)))
    (when (member :right arm-as-list)
      (setf right-reach-poses (subseq right-manipulation-poses 0 2)
            right-grasp-poses (subseq right-manipulation-poses 2 3)
            right-lift-poses (subseq right-manipulation-poses 3)))
    (list left-reach-poses right-reach-poses
          left-grasp-poses right-grasp-poses
          left-lift-poses right-lift-poses)))

(defun extract-place-manipulation-poses (arm left-manipulation-poses right-manipulation-poses)
  "`?arm' can be :left, :right or (:left :right)."
  (let ((arm-as-list (if (listp arm) arm (list arm)))
        (left-manipulation-poses (reverse left-manipulation-poses))
        (right-manipulation-poses (reverse right-manipulation-poses))
        left-reach-poses left-put-poses left-retract-poses
        right-reach-poses right-put-poses right-retract-poses)
    (when (member :left arm-as-list)
      (setf left-reach-poses (subseq left-manipulation-poses 0 2)
            left-put-poses (subseq left-manipulation-poses 2 3)
            left-retract-poses (subseq left-manipulation-poses 3)))
    (when (member :right arm-as-list)
      (setf right-reach-poses (subseq right-manipulation-poses 0 2)
            right-put-poses (subseq right-manipulation-poses 2 3)
            right-retract-poses (subseq right-manipulation-poses 3)))
    (list left-reach-poses right-reach-poses
          left-put-poses right-put-poses
          left-retract-poses right-retract-poses)))

(defun pose->transform-stamped-in-base (pose child-frame-lispy)
  (let ((target-pose-in-base
          (cram-tf:ensure-pose-in-frame
           pose cram-tf:*robot-base-frame* :use-zero-time t))
        (child-frame-rosy
          (roslisp-utilities:rosify-underscores-lisp-name child-frame-lispy)))
    (cram-tf:pose-stamped->transform-stamped target-pose-in-base child-frame-rosy)))


(def-fact-group pick-and-place-plans (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (pick-up ?current-object-desig ?arm
                                                          ?gripper-opening ?effort ?grasp
                                                          ?left-reach-poses ?right-reach-poses
                                                          ?left-grasp-poses ?right-grasp-poses
                                                          ?left-lift-poses ?right-lift-poses))
    ;; extract info from ?action-designator
    (spec:property ?action-designator (:type :picking-up))
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
        (and (lisp-fun man-int:get-object-type-grasps ?object-type ?arm ?grasps)
             (member ?grasp ?grasps)))
    (lisp-fun man-int:get-object-type-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-object-type-gripper-opening ?object-type ?gripper-opening)
    (lisp-fun man-int:get-object-grasping-poses
              ?object-name ?object-type :left ?grasp ?object-transform
              ?left-poses)
    (lisp-fun man-int:get-object-grasping-poses
              ?object-name ?object-type :right ?grasp ?object-transform
              ?right-poses)
    (lisp-fun extract-pick-up-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses
                                 ?left-grasp-poses ?right-grasp-poses
                                 ?left-lift-poses ?right-lift-poses)))

  (<- (desig:action-grounding ?action-designator (place ?current-object-designator
                                                        ?other-object-designator
                                                        ?placement-location-name
                                                        ?arm
                                                        ?gripper-opening
                                                        ?left-reach-poses ?right-reach-poses
                                                        ?left-put-poses ?right-put-poses
                                                        ?left-retract-poses ?right-retract-poses))
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
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        ;; TODO: grasp should be stored in the knowledge base!!
        (and (lisp-fun man-int:get-object-type-grasps ?object-type ?arm ?grasps)
             (member ?grasp ?grasps)))
    (lisp-fun man-int:get-object-type-gripper-opening ?object-type ?gripper-opening)

    ;; take object-pose from action-designator :target otherwise from object-designator pose
    (-> (spec:property ?action-designator (:target ?location-designator))
        (and (desig:current-designator ?location-designator ?current-location-designator)
             (desig:designator-groundings ?current-location-designator ?poses)
             (member ?target-pose ?poses)
             (lisp-fun pose->transform-stamped-in-base ?target-pose ?object-name
                       ?target-transform))
        (and (lisp-fun man-int:get-object-transform ?current-object-designator ?target-transform)
             (lisp-fun man-int:get-object-pose ?current-object-designator ?target-pose)
             (desig:designator :location ((:pose ?target-pose)) ?current-location-designator)))

    ;; placing happens on/in an object
    (or (desig:desig-prop ?current-location-designator (:on ?other-object-designator))
        (desig:desig-prop ?current-location-designator (:in ?other-object-designator))
        (equal ?other-object-designator NIL))
    (-> (desig:desig-prop ?current-location-designator (:attachment ?placement-location-name))
        (true)
        (equal ?placement-location-name NIL))

    (lisp-fun man-int:get-object-grasping-poses
              ?object-name ?object-type :left ?grasp ?target-transform
              ?left-poses)
    (lisp-fun man-int:get-object-grasping-poses
              ?object-name ?object-type :right ?grasp ?target-transform
              ?right-poses)
    (lisp-fun extract-place-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses ?left-put-poses ?right-put-poses
                                 ?left-retract-poses ?right-retract-poses))))
