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

(def-fact-group boxy-place-connect-look-designators (desig:action-grounding)
 

  (<- (desig:action-grounding ?action-designator (place ?arm ?current-object-designator 
                                                        ?left-reach-poses ?right-reach-poses
                                                        ?left-put-poses ?right-put-poses
                                                        ?left-retract-poses ?right-retract-poses))
    (spec:property ?action-designator (:type :placing))
    (-> (spec:property ?action-designator (:arm ?arm))
        (-> (spec:property ?action-designator (:object ?object-designator))
            (or (cpoe:object-in-hand ?object-designator ?arm)
                (and (format "WARNING: Wanted to place an object ~a with arm ~a, ~
                              but it's not in the arm.~%" ?object-designator ?arm)
                     ;; (fail)
                     ))
            (cpoe:object-in-hand ?object-designator ?arm))
        (-> (spec:property ?action-designator (:object ?object-designator))
            (cpoe:object-in-hand ?object-designator ?arm)
            (and (cram-robot-interfaces:robot ?robot)
                 (cram-robot-interfaces:arm ?robot ?arm)
                 (cpoe:object-in-hand ?object-designator ?arm))))
    (once (or (cpoe:object-in-hand ?object-designator ?arm)
              (spec:property ?action-designator (:object ?object-designator))))
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:type ?object-type))
    (spec:property ?current-object-designator (:name ?object-name))
    (spec:property ?action-designator (:on-object ?on-object-designator))
    (desig:current-designator ?on-object-designator ?current-on-object-designator)
    (spec:property ?current-on-object-designator (:type ?on-object-type))
    (spec:property ?current-on-object-designator (:name ?on-object-name))
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
    (lisp-fun pp-plans::extract-place-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses ?left-put-poses ?right-put-poses
                                 ?left-retract-poses ?right-retract-poses)))

  (<- (desig:action-grounding ?action-designator (connect ?arm
                                                          ?current-object-designator
                                                          ?with-object-designator
                                                          ?gripper-opening
                                                          ?left-reach-poses ?right-reach-poses
                                                          ?left-push-poses ?right-push-poses
                                                          ?left-retract-poses ?right-retract-poses))
    (spec:property ?action-designator (:type :connecting))
    (-> (spec:property ?action-designator (:arm ?arm))
        (-> (spec:property ?action-designator (:object ?object-designator))
            (or (cpoe:object-in-hand ?object-designator ?arm)
                (and (format "WARNING: Wanted to place an object ~a with arm ~a, ~
                              but it's not in the arm.~%" ?object-designator ?arm)
                     ;; (fail)
                     ))
            (cpoe:object-in-hand ?object-designator ?arm))
        (-> (spec:property ?action-designator (:object ?object-designator))
            (cpoe:object-in-hand ?object-designator ?arm)
            (and (cram-robot-interfaces:robot ?robot)
                 (cram-robot-interfaces:arm ?robot ?arm)
                 (cpoe:object-in-hand ?object-designator ?arm))))
    (once (or (cpoe:object-in-hand ?object-designator ?arm)
              (spec:property ?action-designator (:object ?object-designator))))
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:type ?object-type))
    (spec:property ?current-object-designator (:name ?object-name))
    (spec:property ?action-designator (:with-object ?with-object-designator))
    (desig:current-designator ?with-object-designator ?current-with-object-designator)
    (spec:property ?current-with-object-designator (:type ?with-object-type))
    (spec:property ?current-with-object-designator (:name ?with-object-name))
    (lisp-fun obj-int:get-object-type-gripper-opening ?object-type ?gripper-opening)
    (lisp-fun obj-int:get-object-transform ?current-with-object-designator
              ?with-object-transform)
    ;; infer missing information
    (obj-int:object-type-grasp ?object-type ?grasp)
    (lisp-fun kr-assembly::get-object-placing-poses ?with-object-name ?with-object-type
              ?object-name ?object-type :left ?grasp
              ?with-object-transform ?left-poses)
    ;; only use the left arm for now
    (equal ?right-poses NIL)
    (lisp-fun pp-plans::extract-place-manipulation-poses ?arm ?left-poses ?right-poses
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
    (equal ?right-goal-pose NIL)))
