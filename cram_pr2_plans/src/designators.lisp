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

(in-package :pr2-plans)

(defun append-pick-up-action-designator (action-designator ?arm
                                         ?left-pregrasp-poses ?right-pregrasp-poses
                                         ?left-grasp-pose ?right-grasp-pose
                                         ?object-grip-effort
                                         ?left-lift-pose
                                         ?right-lift-pose)
  (case ?arm
    (:left (setf ?right-pregrasp-poses nil
                 ?right-grasp-pose nil
                 ?right-lift-pose nil))
    (:right (setf ?left-pregrasp-poses nil
                  ?left-grasp-pose nil
                  ?left-lift-pose nil)))
  (let ((phases (list
                 (an action
                    (to my-reach)
                    (left ?left-pregrasp-poses)
                    (right ?right-pregrasp-poses))
                 (an action
                    (to my-open)
                    (?arm gripper))
                 (an action
                    (to my-grasp)
                    (left ?left-grasp-pose)
                    (right ?right-grasp-pose))
                 (an action
                    (to my-grip)
                    (with ?arm)
                    (effort ?object-grip-effort))
                 (an action
                    (to my-lift)
                    (left ?left-lift-pose)
                    (right ?right-lift-pose)))))
    (copy-designator action-designator :new-description `((:phases ,phases)))))

(defun append-place-action-designator (action-designator ?arm
                                       ?left-put-poses ?right-put-poses
                                       ?left-grasp-poses ?right-grasp-poses)
  (case ?arm
    (:left (setf ?right-grasp-poses nil
                 ?right-put-poses nil))
    (:right (setf ?left-grasp-poses nil
                  ?left-put-poses nil)))
  ;; (setf ?left-grasp-poses (reverse ?left-grasp-poses))
  ;; (setf ?right-grasp-poses (reverse ?right-grasp-poses))
  (let ((phases (list
                 (an action
                     (to my-put)
                     (left ?left-put-poses)
                     (right ?right-put-poses))
                 (an action
                     (to my-open)
                     (?arm gripper))
                 (an action
                     (to my-retract)
                     (left ?left-grasp-poses)
                     (right ?right-grasp-poses)))))
    (copy-designator action-designator :new-description `((:phases ,phases)))))

(def-fact-group pr2-manipulation-plans (action-desig)

  (<- (object-type-grip-maximum-effort :cup 50))
  (<- (object-type-grip-maximum-effort :mondamin 30))
  (<- (object-type-grip-maximum-effort :handle 50))

  (<- (gripper-action-maximum-effort ?object-designator ?maximum-effort)
    (current-designator ?object-designator ?current-object-designator)
    (desig-prop ?current-object-designator (:type ?object-type))
    (once (or (object-type-grip-maximum-effort ?object-type ?maximum-effort)
              (equal ?maximum-effort nil))))

  (<- (object-type-grasp :cutlery :top))
  (<- (object-type-grasp :plate :side))
  (<- (object-type-grasp :bottle :side))
  (<- (object-type-grasp :cup :front))

  (<- (action-desig ?action-designator (perform-phases-in-sequence ?updated-action-designator))
    (or (desig-prop ?action-designator (:to :my-pick-up))
        (desig-prop ?action-designator (:type :my-picking-up)))
    (once (or (desig-prop ?action-designator (:arm ?arm))
              (equal ?arm (:left :right))))
    (desig-prop ?action-designator (:object ?object-designator))
    (current-designator ?object-designator ?current-object-designator)
    (desig-prop ?current-object-designator (:type ?object-type))
    (object-type-grasp ?object-type ?grasp)
    ;; (or (equal ?object-type :plate)
    ;;     (equal ?object-type :cutlery)
    ;;     (equal ?object-type :cup)
    ;;     (equal ?object-type :bottle))
    ;; so we have (an action (to pick-up) (object (an object (type cutlery))))
    ;; now we need to add the phases with the corresponding via-points and efforts
    ;; find the missing info
    (lisp-fun get-object-grasp-pose ?current-object-designator :left ?grasp
              ?left-grasp-pose)
    (lisp-fun get-object-grasp-pose ?current-object-designator :right ?grasp
              ?right-grasp-pose)
    ;;
    (lisp-fun get-object-type-pregrasp-pose ?object-type ?left-grasp-pose :left ?grasp
              ?left-pregrasp-pose)
    (lisp-fun get-object-type-pregrasp-pose ?object-type ?right-grasp-pose :right ?grasp
              ?right-pregrasp-pose)
    ;;
    (lisp-fun get-object-type-2nd-pregrasp-pose ?object-type ?left-grasp-pose :left ?grasp
              ?left-2nd-pregrasp-pose)
    (lisp-fun get-object-type-2nd-pregrasp-pose ?object-type ?right-grasp-pose :right ?grasp
              ?right-2nd-pregrasp-pose)
    ;;
    (lisp-fun get-object-grasp-lift-pose ?left-grasp-pose ?left-lift-pose)
    (lisp-fun get-object-grasp-lift-pose ?right-grasp-pose ?right-lift-pose)
    ;;
    (equal ?left-grasp-poses (?left-2nd-pregrasp-pose ?left-grasp-pose))
    (equal ?right-grasp-poses (?right-2nd-pregrasp-pose ?right-grasp-pose))
    ;;
    (gripper-action-maximum-effort ?object-designator ?maximum-effort)
    ;; create new designator with updated appended action-description
    (lisp-fun append-pick-up-action-designator ?action-designator ?arm
              ?left-pregrasp-pose ?right-pregrasp-pose
              ?left-grasp-poses ?right-grasp-poses
              ?maximum-effort
              ?left-lift-pose ?right-lift-pose ?updated-action-designator))

  (<- (action-desig ?action-designator (perform-phases-in-sequence ?updated-action-designator))
    (or (desig-prop ?action-designator (:to :my-place))
        (desig-prop ?action-designator (:type :my-placing)))
    (once (or (desig-prop ?action-designator (:arm ?arm))
              (equal ?arm (:left :right))))
    (desig-prop ?action-designator (:object ?object-designator))
    (current-designator ?object-designator ?current-object-designator)
    (desig-prop ?current-object-designator (:type ?object-type))
    (object-type-grasp ?object-type ?grasp)
    (desig-prop ?action-designator (:at ?object-pose))
    ;; (or (equal ?object-type :plate)
    ;;     (equal ?object-type :cutlery)
    ;;     (equal ?object-type :cup)
    ;;     (equal ?object-type :bottle))
    ;; so we have (an action (to put-down) (object ...) (at some-location-or-pose-stamped))
    ;; now we need to add the phases with the corresponding via-points and efforts
    ;; find the missing info
    (lisp-fun get-object-type-grasp-pose ?object-type ?object-pose :left ?grasp
              ?left-grasp-pose)
    (lisp-fun get-object-type-grasp-pose ?object-type ?object-pose :right ?grasp
              ?right-grasp-pose)
    ;;
    (lisp-fun get-object-grasp-lift-pose ?left-grasp-pose ?left-lift-pose)
    (lisp-fun get-object-grasp-lift-pose ?right-grasp-pose ?right-lift-pose)
    ;;
    (equal ?left-put-poses (?left-lift-pose ?left-grasp-pose))
    (equal ?right-put-poses (?right-lift-pose ?right-grasp-pose))
    ;;
    (lisp-fun get-object-type-pregrasp-pose ?object-type ?left-grasp-pose :left ?grasp
              ?left-pregrasp-pose)
    (lisp-fun get-object-type-pregrasp-pose ?object-type ?right-grasp-pose :right ?grasp
              ?right-pregrasp-pose)
    ;;
    (lisp-fun get-object-type-2nd-pregrasp-pose ?object-type ?left-grasp-pose :left ?grasp
              ?left-2nd-pregrasp-pose)
    (lisp-fun get-object-type-2nd-pregrasp-pose ?object-type ?right-grasp-pose :right ?grasp
              ?right-2nd-pregrasp-pose)
    ;;
    (equal ?left-grasp-poses (?left-grasp-pose ?left-2nd-pregrasp-pose ?left-pregrasp-pose))
    (equal ?right-grasp-poses (?right-grasp-pose ?right-2nd-pregrasp-pose ?right-pregrasp-pose))
    ;; create new designator with updated appended action-description
    (lisp-fun append-place-action-designator ?action-designator ?arm
              ?left-put-poses ?right-put-poses
              ?left-grasp-poses ?right-grasp-poses
              ?updated-action-designator))

   (<- (action-desig ?action-designator (move-arms-in-sequence ?left-poses ?right-poses))
    (or (desig-prop ?action-designator (:to :my-reach))
        (desig-prop ?action-designator (:to :my-lift))
        (desig-prop ?action-designator (:to :my-retract))
        (desig-prop ?action-designator (:to :my-put)))
    (once (or (desig-prop ?action-designator (:left ?left-poses))
              (equal ?left-poses nil)))
    (once (or (desig-prop ?action-designator (:right ?right-poses))
              (equal ?right-poses nil))))

  (<- (action-desig ?action-designator (open-gripper ?left-or-right))
    (desig-prop ?action-designator (:to :my-open))
    (desig-prop ?action-designator (?left-or-right :gripper)))

  (<- (action-desig ?action-designator (grasp ?left-grasp-poses ?right-grasp-poses))
    (desig-prop ?action-designator (:to :my-grasp))
    (once (or (desig-prop ?action-designator (:left ?left-grasp-poses))
              (equal ?left-grasp-poses nil)))
    (once (or (desig-prop ?action-designator (:right ?right-grasp-poses))
              (equal ?right-grasp-poses nil))))

  (<- (action-desig ?action-designator (grip ?left-or-right ?object-grip-effort))
    (desig-prop ?action-designator (:to :my-grip))
    (desig-prop ?action-designator (:with ?left-or-right))
    (desig-prop ?action-designator (:effort ?object-grip-effort))))
