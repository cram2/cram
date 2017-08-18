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

(in-package :pr2-pp-plans)

(defun extract-pick-up-manipulation-poses (arm left-manipulation-poses right-manipulation-poses)
  "`?arm' can be :left, :right or (:left :right)."
  (let ((arm-as-list (if (listp arm) arm (list arm)))
        left-reach-poses left-lift-poses
        right-reach-poses right-lift-poses)
    (when (member :left arm-as-list)
      (setf left-reach-poses (subseq left-manipulation-poses 0 3)
            left-lift-poses (subseq left-manipulation-poses 3)))
    (when (member :right arm-as-list)
      (setf right-reach-poses (subseq right-manipulation-poses 0 3)
            right-lift-poses (subseq right-manipulation-poses 3)))
    (list left-reach-poses right-reach-poses left-lift-poses right-lift-poses)))

(defun extract-place-manipulation-poses (arm left-manipulation-poses right-manipulation-poses)
  "`?arm' can be :left, :right or (:left :right)."
  (let ((arm-as-list (if (listp arm) arm (list arm)))
        left-reach-poses left-put-poses left-retract-poses
        right-reach-poses right-put-poses right-retract-poses)
    (when (member :left arm-as-list)
      (setf left-reach-poses (subseq left-manipulation-poses 0 1)
            left-put-poses (subseq left-manipulation-poses 1 2)
            left-retract-poses (subseq left-manipulation-poses 2)))
    (when (member :right arm-as-list)
      (setf right-reach-poses (subseq right-manipulation-poses 0 1)
            right-put-poses (subseq right-manipulation-poses 1 2)
            right-retract-poses (subseq right-manipulation-poses 2)))
    (list left-reach-poses right-reach-poses left-put-poses right-put-poses
                    left-retract-poses right-retract-poses)))


;; (def-fact-group pr2-pick-and-place-parameters ()

;;   (<- (object-type-grip-maximum-effort :cup 50))
;;   (<- (object-type-grip-maximum-effort :bottle 60))
;;   (<- (object-type-grip-maximum-effort :plate 100))
;;   (<- (object-type-grip-maximum-effort :cutlery 100))
;;   (<- (object-type-grip-maximum-effort :fork 100))
;;   (<- (object-type-grip-maximum-effort :knife 100))

;;   (<- (gripper-action-maximum-effort ?object-designator ?maximum-effort)
;;     (current-designator ?object-designator ?current-object-designator)
;;     (desig-prop ?current-object-designator (:type ?object-type))
;;     (once (or (object-type-grip-maximum-effort ?object-type ?maximum-effort)
;;               (equal ?maximum-effort nil))))

;;   (<- (object-type-grasp :cutlery :top))
;;   (<- (object-type-grasp :fork :top))
;;   (<- (object-type-grasp :knife :top))
;;   (<- (object-type-grasp :plate :side))
;;   (<- (object-type-grasp :bottle :side))
;;   (<- (object-type-grasp :cup :front)))


(def-fact-group pick-and-place-plans (action-grounding)
  (<- (action-grounding ?action-designator (pick-up ?arm ?current-object-desig ?grasp
                                                    ?gripper-opening ?effort
                                                    ?left-reach?updated-action-designator
                                                    ?current-object-desig
                                                    ?arm ?grasp))
    ;; extract info from ?action-designator
    (property ?action-designator (:type :picking-up))
    (property ?action-designator (:object ?object-designator))
    (current-designator ?object-designator ?current-object-desig)
    (property ?current-object-desig (:type ?object-type))
    (once (or (property ?action-designator (:arm ?arm))
              (equal ?arm (:left :right)))) ; default value of ?arm when not given
    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    (object-type-grasp ?object-type ?grasp)
    (gripper-action-maximum-effort ?object-designator ?maximum-effort)
    (lisp-fun cram-robosherlock:get-object-pose ?current-object-desig ?object-pose)
    (lisp-fun get-object-manipulation-poses ?object-type ?object-pose :left ?grasp ?left-poses)
    (lisp-fun get-object-manipulation-poses ?object-type ?object-pose :right ?grasp ?right-poses)
    ;; feed the inferred information into the ?updated-action-designator
    (lisp-fun append-pick-up-action-designator
              ?action-designator ?arm ?maximum-effort ?left-poses ?right-poses
              ?updated-action-designator))

  (<- (action-grounding ?action-designator (place ?updated-action-designator ?arm))
    (property ?action-designator (:type :placing))
    (property ?action-designator (:arm ?arm))
    (once (or (cpoe:object-in-hand ?object-designator ?arm)
              (property ?action-designator (:object ?object-designator))))
    (current-designator ?object-designator ?current-object-designator)
    (property ?current-object-designator (:type ?object-type))
    ;; infer missing information
    (object-type-grasp ?object-type ?grasp)
    ;; take object-pose from action-designator target otherwise from object-designator pose
    (-> (property ?action-designator (:target ?location))
        (and (desig:current-designator ?location ?current-location-designator)
             (desig:designator-groundings ?current-location-designator ?poses)
             (member ?object-pose ?poses))
        (lisp-fun cram-robosherlock:get-object-pose ?current-object-designator ?object-pose))
    (lisp-fun get-object-manipulation-poses ?object-type ?object-pose :left ?grasp ?left-poses)
    (lisp-fun get-object-manipulation-poses ?object-type ?object-pose :right ?grasp ?right-poses)
    ;; create new designator with updated appended action-description
    (lisp-fun append-place-action-designator ?action-designator ?arm ?left-poses ?right-poses
              ?updated-action-designator)))



;; (defun append-pick-up-action-designator (action-designator ?arm ?object-grip-effort
;;                                          left-manipulation-poses right-manipulation-poses)
;;   "`?arm' can be :left, :right or (:left :right)."
;;   (let ((arm-as-list (if (listp ?arm) ?arm (list ?arm)))
;;         ?left-grasp-poses ?left-lift-pose
;;         ?right-grasp-poses ?right-lift-pose)
;;     (when (member :left arm-as-list)
;;       (setf ?left-grasp-poses (subseq left-manipulation-poses 0 3)
;;             ?left-lift-pose (subseq left-manipulation-poses 3)))
;;     (when (member :right arm-as-list)
;;       (setf ?right-grasp-poses (subseq right-manipulation-poses 0 3)
;;             ?right-lift-pose (subseq right-manipulation-poses 3)))

;;     (copy-designator action-designator
;;                      :new-description
;;                      `((:phases ,(list
;;                                   (an action
;;                                       (type opening)
;;                                       (gripper ?arm))
;;                                   (an action
;;                                       (type reaching)
;;                                       (left ?left-grasp-poses)
;;                                       (right ?right-grasp-poses))
;;                                   (an action
;;                                       (type gripping)
;;                                       (arm ?arm)
;;                                       (effort ?object-grip-effort))
;;                                   (an action
;;                                       (type lifting)
;;                                       (left ?left-lift-pose)
;;                                       (right ?right-lift-pose))))))))

;; (defun append-place-action-designator (action-designator ?arm
;;                                        left-manipulation-poses right-manipulation-poses)
;;   "`?arm' can be :left, :right or (:left :right)."
;;   (let ((arm-as-list (if (listp ?arm) ?arm (list ?arm)))
;;         (left-manipulation-poses (reverse left-manipulation-poses))
;;         (right-manipulation-poses (reverse right-manipulation-poses))
;;         ?left-put-poses ?left-grasp-poses
;;         ?right-put-poses ?right-grasp-poses)
;;     (when (member :left arm-as-list)
;;       (setf ?left-put-poses (subseq left-manipulation-poses 0 2)
;;             ?left-grasp-poses (subseq left-manipulation-poses 2)))
;;     (when (member :right arm-as-list)
;;       (setf ?right-put-poses (subseq right-manipulation-poses 0 2)
;;             ?right-grasp-poses (subseq right-manipulation-poses 2)))

;;     (copy-designator action-designator
;;                      :new-description
;;                      `((:phases ,(list
;;                                   (an action
;;                                       (type putting)
;;                                       (left ?left-put-poses)
;;                                       (right ?right-put-poses))
;;                                   (an action
;;                                       (type opening)
;;                                       (gripper ?arm))
;;                                   (an action
;;                                       (type retracting)
;;                                       (left ?left-grasp-poses)
;;                                       (right ?right-grasp-poses))))))))
