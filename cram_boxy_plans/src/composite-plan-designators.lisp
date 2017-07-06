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

(defun append-pick-up-action-designator (action-designator ?arm ?object-grip-effort
                                         left-manipulation-poses right-manipulation-poses)
  "`?arm' can be :left, :right or (:left :right)."
  (let ((arm-as-list (if (listp ?arm) ?arm (list ?arm)))
        ?left-reach-poses ?left-lift-pose
        ?right-reach-poses ?right-lift-pose)
    (when (member :left arm-as-list)
      (setf ?left-reach-poses (subseq left-manipulation-poses 0 3)
            ?left-lift-pose (subseq left-manipulation-poses 3)))
    (when (member :right arm-as-list)
      (setf ?right-reach-poses (subseq right-manipulation-poses 0 3)
            ?right-lift-pose (subseq right-manipulation-poses 3)))

    (desig:copy-designator action-designator
                           :new-description
                           `((:phases ,(list
                                        (desig:an action
                                                  (type opening)
                                                  (gripper ?arm))
                                        (desig:an action
                                                  (type reaching)
                                                  (left ?left-reach-poses)
                                                  (right ?right-reach-poses))
                                        (desig:an action
                                                  (type gripping)
                                                  (arm ?arm)
                                                  (effort ?object-grip-effort))
                                        (desig:an action
                                                  (type lifting)
                                                  (left ?left-lift-pose)
                                                  (right ?right-lift-pose))))))))

(defun append-place-action-designator (action-designator ?arm
                                       left-manipulation-poses right-manipulation-poses)
  "`?arm' can be :left, :right or (:left :right)."
  (let ((arm-as-list (if (listp ?arm) ?arm (list ?arm)))
        (left-manipulation-poses (reverse left-manipulation-poses))
        (right-manipulation-poses (reverse right-manipulation-poses))
        ?left-reach-poses ?left-put-pose ?left-retract-poses
        ?right-reach-poses ?right-put-pose ?right-retract-poses)
    (when (member :left arm-as-list)
      (setf ?left-reach-poses (subseq left-manipulation-poses 0 1)
            ?left-put-pose (second left-manipulation-poses)
            ?left-retract-poses (subseq left-manipulation-poses 2)))
    (when (member :right arm-as-list)
      (setf ?right-reach-poses (subseq right-manipulation-poses 0 1)
            ?right-put-pose (second right-manipulation-poses)
            ?right-retract-poses (subseq right-manipulation-poses 2)))

    (desig:copy-designator action-designator
                           :new-description
                           `((:phases ,(list
                                        (desig:an action
                                                  (type reaching)
                                                  (left ?left-reach-poses)
                                                  (right ?right-reach-poses))
                                        (desig:an action
                                                  (type putting)
                                                  (left ?left-put-pose)
                                                  (right ?right-put-pose))
                                        (desig:an action
                                                  (type releasing)
                                                  (gripper ?arm))
                                        (desig:an action
                                                  (type retracting)
                                                  (left ?left-retract-poses)
                                                  (right ?right-retract-poses))))))))

(defun append-connect-action-designator (action-designator ?arm
                                         left-manipulation-poses right-manipulation-poses)
  "`?arm' can be :left, :right or (:left :right)."
  (let ((arm-as-list (if (listp ?arm) ?arm (list ?arm)))
        (left-manipulation-poses (reverse left-manipulation-poses))
        (right-manipulation-poses (reverse right-manipulation-poses))
        ?left-reach-poses ?left-push-pose ?left-retract-poses
        ?right-reach-poses ?right-push-pose ?right-retract-poses)
    (when (member :left arm-as-list)
      (setf ?left-reach-poses (subseq left-manipulation-poses 0 1)
            ?left-push-pose (second left-manipulation-poses)
            ?left-retract-poses (subseq left-manipulation-poses 2)))
    (when (member :right arm-as-list)
      (setf ?right-reach-poses (subseq right-manipulation-poses 0 1)
            ?right-push-pose (second right-manipulation-poses)
            ?right-retract-poses (subseq right-manipulation-poses 2)))

    (desig:copy-designator action-designator
                           :new-description
                           `((:phases ,(list
                                        (desig:an action
                                                  (type reaching)
                                                  (left ?left-reach-poses)
                                                  (right ?right-reach-poses))
                                        (desig:an action
                                                  (type pushing)
                                                  (left ?left-push-pose)
                                                  (right ?right-push-pose))
                                        (desig:an action
                                                  (type releasing)
                                                  (gripper ?arm))
                                        (desig:an action
                                                  (type retracting)
                                                  (left ?left-retract-poses)
                                                  (right ?right-retract-poses))))))))

;;;;;;;;;;;;; TODO: think about context objects -- do we really need them?

(def-fact-group pr2-pick-and-place-plans (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (pick-up ?updated-action-designator
                                                          ?current-object-desig
                                                          ?arm ?grasp))
    ;; extract info from ?action-designator
    (property ?action-designator (:type :picking-up))
    (property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (property ?current-object-desig (:type ?object-type))
    (property ?action-designator (:from ?contect-object-type))
    (once (or (property ?action-designator (:arm ?arm))
              (equal ?arm :left))) ; default value of ?arm when not given
    ;; infer missing information like ?grasp type, gripping ?effort, manipulation poses
    (lisp-fun kr-belief::get-object-type-grasp ?object-type ?grasp)
    (lisp-fun kr-belief::get-object-type-gripping-effort ?object-type ?effort)
    (lisp-fun cram-robosherlock:get-object-transform ?current-object-desig ?object-transform)
    (lisp-fun kr-belief::get-object-manipulation-poses ?object-type ?context-object-type
              :left ?grasp ?object-transform ?left-poses)
    ;; (lisp-fun kr-belief::get-object-manipulation-poses ?object-type ?context-object-type
    ;;           :right ?grasp ?object-pose ?right-poses)
    (equal ?right-poses NIL) ; for now only use left arm
    ;; feed the inferred information into the ?updated-action-designator
    (lisp-fun append-pick-up-action-designator
              ?action-designator ?arm ?effort ?left-poses ?right-poses
              ?updated-action-designator))

  (<- (desig:action-grounding ?action-designator (place ?updated-action-designator ?arm))
    (property ?action-designator (:type :placing))
    (property ?action-designator (:arm ?arm))
    (once (or (cpoe:object-in-hand ?object-designator ?arm)
              (property ?action-designator (:object ?object-designator))))
    (desig:current-designator ?object-designator ?current-object-designator)
    (property ?current-object-designator (:type ?object-type))
    ;; infer missing information
    (lisp-fun kr-belief::get-object-type-grasp ?object-type ?grasp)
    ;; take object-pose from action-designator target otherwise from object-designator pose
    (-> (property ?action-designator (:target ?location))
        ;;;;;;;;;;;;;;;;;;;; TODO TARGET SHOULD BE INFERRED FROM SYMBOLIC NAME
        (and (desig:current-designator ?location ?current-location-designator)
             (desig:designator-groundings ?current-location-designator ?poses)
             (member ?object-pose ?poses))
        (lisp-fun cram-robosherlock:get-object-transform ?current-object-designator
                  ?object-transform))
    (lisp-fun kr-belief::get-object-manipulation-poses ?object-type :chassie-holder :left ?grasp
               ?object-transform ?left-poses)
    ;; (lisp-fun kr-belief::get-object-manipulation-poses ?object-type :axle-holder :right ?grasp
    ;;           ?object-pose ?right-poses)
    (equal ?right-poses NIL) ; only use the left arm for now
    ;; create new designator with updated appended action-description
    (lisp-fun append-place-action-designator ?action-designator ?arm ?left-poses ?right-poses
              ?updated-action-designator))

  (<- (desig:action-grounding ?action-designator (place ?updated-action-designator ?arm))
    (property ?action-designator (:type :connecting))
    (property ?action-designator (:arm ?arm))
    (once (or (cpoe:object-in-hand ?object-designator ?arm)
              (property ?action-designator (:object ?object-designator))))
    (desig:current-designator ?object-designator ?current-object-designator)
    (property ?current-object-designator (:type ?object-type))
    (desig:desig-prop ?action-designator (:with-object ?context-object-type))
    ;; infer missing information
    (lisp-fun kr-belief::get-object-type-grasp ?object-type ?grasp)
    ;; take object-pose from action-designator target otherwise from object-designator pose
    (-> (property ?action-designator (:target ?location))
        ;;;;;;;;;;;;;;;;;;;; TODO TARGET SHOULD BE INFERRED FROM SYMBOLIC NAME
        (and (desig:current-designator ?location ?current-location-designator)
             (desig:designator-groundings ?current-location-designator ?poses)
             (member ?object-pose ?poses))
        (lisp-fun cram-robosherlock:get-object-transform ?current-object-designator
                  ?object-transform))
    (lisp-fun kr-belief::get-object-manipulation-poses ?object-type ?context-object-type
              :left ?grasp ?object-transform ?left-poses)
    ;; (lisp-fun kr-belief::get-object-manipulation-poses ?object-type ?context-object-type
    ;;           :right ?grasp ?object-pose ?right-poses)
    (equal ?right-poses NIL) ; only use the left arm for now
    ;; create new designator with updated appended action-description
    (lisp-fun append-place-action-designator ?action-designator ?arm ?left-poses ?right-poses
              ?updated-action-designator)))
