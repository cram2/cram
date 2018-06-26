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

(defun extract-pick-up-manipulation-poses (arm left-manipulation-poses right-manipulation-poses)
  "`?arm' can be :left, :right or (:left :right)."
  (let ((arm-as-list (if (listp arm) arm (list arm)))
        left-reach-poses left-lift-poses
        right-reach-poses right-lift-poses)
    (when (member :left arm-as-list)
      (setf left-reach-poses (subseq left-manipulation-poses 0 2)
            left-lift-poses (subseq left-manipulation-poses 2)))
    (when (member :right arm-as-list)
      (setf right-reach-poses (subseq right-manipulation-poses 0 2)
            right-lift-poses (subseq right-manipulation-poses 2)))
    (list left-reach-poses right-reach-poses left-lift-poses right-lift-poses)))

(defun extract-place-manipulation-poses (arm left-manipulation-poses right-manipulation-poses)
  "`?arm' can be :left, :right or (:left :right)."
  (let ((arm-as-list (if (listp arm) arm (list arm)))
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
    (list left-reach-poses right-reach-poses left-put-poses right-put-poses
          left-retract-poses right-retract-poses)))

;; (defun cram-object-interfaces:get-object-transform (object-designator)
;;   (let* ((object-type (desig:desig-prop-value object-designator :type))
;;          (object-frame (concatenate 'string
;;                                     (remove #\- (string-capitalize (symbol-name object-type)))
;;                                     "1")))
;;     (cl-transforms-stamped:lookup-transform
;;      cram-tf:*transformer*
;;      cram-tf:*robot-base-frame*
;;      object-frame
;;      :time 0.0
;;      :timeout cram-tf:*tf-default-timeout*)))

(def-fact-group boxy-pick-place-connect-look-designators (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (pick-up ?arm ?current-object-desig ?grasp
                                                          ?gripper-opening ?effort
                                                          ?left-reach-poses ?right-reach-poses
                                                          ?left-lift-poses ?right-lift-poses))
    ;; extract info from ?action-designator
    (property ?action-designator (:type :picking-up))
    (property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (property ?current-object-desig (:type ?object-type))
    (property ?current-object-desig (:name ?object-name))
    (property ?action-designator (:arm ?arm))
    ;; infer missing information like ?grasp type, gripping ?effort, manipulation poses
    (obj-int:object-type-grasp ?object-type ?grasp)
    (lisp-fun kr-assembly::get-object-type-gripping-effort ?object-type ?effort)
    (lisp-fun kr-assembly::get-object-type-gripper-opening ?object-type ?gripper-opening)
    (lisp-fun cram-object-interfaces:get-object-transform ?current-object-desig ?object-transform)
    (lisp-fun kr-assembly::get-object-grasping-poses
              ?object-name ?object-type :left ?grasp ?object-transform
              ?left-poses)
    (lisp-fun kr-assembly::get-object-grasping-poses
              ?object-name ?object-type :right ?grasp ?object-transform
              ?right-poses)
    (lisp-fun extract-pick-up-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses ?left-lift-poses ?right-lift-poses)))

  (<- (desig:action-grounding ?action-designator (place ?arm ?current-object-designator 
                                                        ?left-reach-poses ?right-reach-poses
                                                        ?left-put-poses ?right-put-poses
                                                        ?left-retract-poses ?right-retract-poses))
    (property ?action-designator (:type :placing))
    (property ?action-designator (:arm ?arm))
    (once (or (cpoe:object-in-hand ?object-designator ?arm)
              (property ?action-designator (:object ?object-designator))))
    (desig:current-designator ?object-designator ?current-object-designator)
    (property ?current-object-designator (:type ?object-type))
    (property ?current-object-designator (:name ?object-name))
    (property ?action-designator (:on-object ?on-object-designator))
    (desig:current-designator ?on-object-designator ?current-on-object-designator)
    (property ?current-on-object-designator (:type ?on-object-type))
    (property ?current-on-object-designator (:name ?on-object-name))
    (lisp-fun cram-object-interfaces:get-object-transform ?current-on-object-designator
              ?on-object-transform)
    ;; infer missing information
    (obj-int:object-type-grasp ?object-type ?grasp)
    (lisp-fun kr-assembly::get-object-placing-poses ?on-object-name ?on-object-type
              ?object-name ?object-type :left ?grasp
              ?on-object-transform ?left-poses)
    (lisp-fun kr-assembly::get-object-placing-poses ?on-object-name ?on-object-type
              ?object-name ?object-type :right ?grasp
              ?on-object-transform ?right-poses)
    (lisp-fun extract-place-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses ?left-put-poses ?right-put-poses
                                 ?left-retract-poses ?right-retract-poses)))

  (<- (desig:action-grounding ?action-designator (connect ?arm
                                                          ?current-object-designator
                                                          ?with-object-designator
                                                          ?gripper-opening
                                                          ?left-reach-poses ?right-reach-poses
                                                          ?left-push-poses ?right-push-poses
                                                          ?left-retract-poses ?right-retract-poses))
    (property ?action-designator (:type :connecting))
    (property ?action-designator (:arm ?arm))
    (once (or (cpoe:object-in-hand ?object-designator ?arm)
              (property ?action-designator (:object ?object-designator))))
    (desig:current-designator ?object-designator ?current-object-designator)
    (property ?current-object-designator (:type ?object-type))
    (property ?current-object-designator (:name ?object-name))
    (property ?action-designator (:with-object ?with-object-designator))
    (desig:current-designator ?with-object-designator ?current-with-object-designator)
    (property ?current-with-object-designator (:type ?with-object-type))
    (property ?current-with-object-designator (:name ?with-object-name))
    (lisp-fun kr-assembly::get-object-type-gripper-opening ?object-type ?gripper-opening)
    (lisp-fun cram-object-interfaces:get-object-transform ?current-with-object-designator
              ?with-object-transform)
    ;; infer missing information
    (obj-int:object-type-grasp ?object-type ?grasp)
    (lisp-fun kr-assembly::get-object-placing-poses ?with-object-name ?with-object-type
              ?object-name ?object-type :left ?grasp
              ?with-object-transform ?left-poses)
    ;; only use the left arm for now
    (equal ?right-poses NIL)
    (lisp-fun extract-place-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses ?left-push-poses ?right-push-poses
               ?left-retract-poses ?right-retract-poses)))

  (<- (desig:action-grounding ?action-designator (look ?left-goal-pose ?right-goal-pose))
    (property ?action-designator (:type :looking))
    (property ?action-designator (:camera :wrist))
    (property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-designator)
    (lisp-fun cram-object-interfaces:get-object-transform
              ?current-object-designator ?object-transform)
    ;; infer missing information
    (lisp-fun kr-assembly::get-object-look-pose :left ?object-transform ?left-goal-pose)
    ;; the only wrist camera is on left arm
    (equal ?right-goal-pose NIL))

  (<- (desig:action-grounding ?action-designator (detect ?object-designator))
    (property ?action-designator (:type :detecting))
    (property ?action-designator (:object ?object-designator))))
