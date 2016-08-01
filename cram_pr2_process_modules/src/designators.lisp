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

(in-package :pr2-pms)

(def-fact-group pr2-navigation-actions (action-desig
                                        matching-process-module
                                        available-process-module)

  (<- (matching-process-module ?action-designator pr2-base-pm)
    (or (and (desig-prop ?action-designator (:type :going))
             (desig-prop ?action-designator (:destination ?_)))
        (and (desig-prop ?action-designator (:to :go))
             (desig-prop ?action-designator (:to ?where))
             (not (equal ?where :go)))))

  (<- (action-desig ?action-designator (drive ?pose))
    (desig-prop ?action-designator (:type :going))
    (desig-prop ?action-designator (:destination ?where))
    (-> (lisp-type ?where designator)
        (desig-location-prop ?where ?pose)
        (or (cl-transforms:pose ?pose ?where)
            (equal ?where ?pose))))

  (<- (action-desig ?action-designator (drive ?pose))
    (desig-prop ?action-designator (:to :go))
    (desig-prop ?action-designator (:to ?where))
    (not (equal ?where :go))
    (-> (lisp-type ?where designator)
        (desig-location-prop ?where ?pose)
        (or (cram-tf:pose ?pose ?where)
            (equal ?where ?pose))))

  (<- (available-process-module pr2-base-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-ptu-actions (action-desig
                                 matching-process-module
                                 available-process-module)

  (<- (matching-process-module ?action-designator pr2-ptu-pm)
    (or (and (desig-prop ?action-designator (:type :looking))
             (or (desig-prop ?action-designator (:pose ?_))
                 (desig-prop ?action-designator (:location ?_))
                 (desig-prop ?action-designator (:object ?_))))
        (and (desig-prop ?action-designator (:to :look))
             (desig-prop ?action-designator (:at ?_)))))

  (<- (action-desig ?action-designator (look-at ?pose))
    (desig-prop ?action-designator (:type :looking))
    (or (desig-prop ?action-designator (:pose ?pose))
        (and (or (desig-prop ?action-designator (:location ?obj-or-loc))
                 (desig-prop ?action-designator (:object ?obj-or-loc)))
             (desig-location-prop ?obj-or-loc ?pose))))

  (<- (action-desig ?action-designator (look-at ?pose))
    (desig-prop ?action-designator (:to :look))
    (desig-prop ?action-designator (:at ?where))
    (-> (lisp-type ?where designator)
        (desig-location-prop ?where ?pose)
        (or (cram-tf:pose ?pose ?where)
            (equal ?where ?pose))))

  (<- (available-process-module pr2-ptu-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-perception-actions (action-desig
                                        matching-process-module
                                        available-process-module)

  (<- (matching-process-module ?action-designator pr2-perception-pm)
    (or (desig-prop ?action-designator (:type :detecting))
        (desig-prop ?action-designator (:to :detect)))
    (desig-prop ?action-designator (:object ?object-designator))
    ;; (current-designator ?object-designator ?current-object-designator)
    ;; (desig-prop ?current-object-designator (:type ?object-type))
    )

  (<- (action-desig ?action-designator (detect ?object-properties ?quantifier))
    (or (desig-prop ?action-designator (:type :detecting))
        (desig-prop ?action-designator (:to :detect)))
    (desig-prop ?action-designator (:object ?object-designator))
    (-> (lisp-type ?object-designator object-designator)
        (and (current-designator ?object-designator ?current-object-designator)
             ;; (desig-description ?current-object-designator (?object-properties . ?_))
             (desig-description ?current-object-designator ?object-properties)
             ;; somehow desig-description gives ((())) back, i dug deep still nothing
             )
        (prolog:equal ?object-properties ((:type ?object-designator))))
    (equal ?quantifier :an) ; this should be done more smartly :)
    )

  (<- (available-process-module pr2-perception-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-gripper-actions (action-desig
                                     matching-process-module
                                     available-process-module)

  (<- (matching-process-module ?action-designator pr2-grippers-pm)
    (or (and (or (desig-prop ?action-designator (:type :gripping))
                 (desig-prop ?action-designator (:to :grip)))
             (desig-prop ?action-designator (:object ?object-designator))
             (or (desig-prop ?action-designator (:gripper ?which-gripper))
                 (desig-prop ?action-designator (:with ?which-gripper))))
        (and (or (desig-prop ?action-designator (:type :opening))
                 (desig-prop ?action-designator (:type :closing)))
             (desig-prop ?action-designator (:gripper ?_)))
        (and (or (desig-prop ?action-designator (:to :open))
                 (desig-prop ?action-designator (:to :close)))
             (desig-prop ?action-designator (?_ :gripper)))))

  (<- (action-desig ?action-designator (gripper-action :open ?which-gripper))
    (desig-prop ?action-designator (:type :opening))
    (desig-prop ?action-designator (:gripper ?which-gripper)))

  (<- (action-desig ?action-designator (gripper-action :open ?which-gripper))
    (desig-prop ?action-designator (:to :open))
    (desig-prop ?action-designator (?which-gripper :gripper)))

  (<- (action-desig ?action-designator (gripper-action :close ?which-gripper))
    (desig-prop ?action-designator (:type :closing))
    (desig-prop ?action-designator (:gripper ?which-gripper)))

  (<- (action-desig ?action-designator (gripper-action :close ?which-gripper))
    (desig-prop ?action-designator (:to :close))
    (desig-prop ?action-designator (?which-gripper :gripper)))

  (<- (%gripper-action-solution ?which-gripper ?object-designator ?solution)
    (current-designator ?object-designator ?current-object-designator)
    (desig-prop ?current-object-designator (:type ?object-type))
    (cram-pr2-description::robot ?robot)
    (-> (cram-pr2-description::object-type-grip-maximum-effort
         ?robot ?object-type ?maximum-effort)
        (equal ?solution (gripper-action :grip ?which-gripper ?maximum-effort))
        (equal ?solution (gripper-action :grip ?which-gripper nil))))

  (<- (action-desig ?action-designator ?solution)
    (desig-prop ?action-designator (:type :gripping))
    (desig-prop ?action-designator (:gripper ?which-gripper))
    (desig-prop ?action-designator (:object ?object-designator))
    (%gripper-action-solution ?which-gripper ?object-designator ?solution))

  (<- (action-desig ?action-designator ?solution)
    (desig-prop ?action-designator (:to :grip))
    (desig-prop ?action-designator (:object ?object-designator))
    (desig-prop ?action-designator (:with ?which-gripper))
    (%gripper-action-solution ?which-gripper ?object-designator ?solution))

  (<- (available-process-module pr2-grippers-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-arm-actions (action-desig
                                 matching-process-module
                                 available-process-module)

  (<- (matching-process-module ?action-designator pr2-arms-pm)
    (or (and (desig-prop ?action-designator (:type :moving))
             (desig-prop ?action-designator (:arm ?_)))
        (and (desig-prop ?action-designator (:to :move))
             (or (desig-prop ?action-designator (?_ :arm))
                 (desig-prop ?action-designator (?_ :arms))))))

  (<- (matching-process-module ?action-designator pr2-arms-pm)
    (or (and (desig-prop ?action-designator (:type :moving))
             (desig-prop ?action-designator (:arm ?_))
             (desig-prop ?action-designator (:goal ?_)))
        (and (desig-prop ?action-designator (:to :move))
             (desig-prop ?action-designator (?_ :arm))
             (desig-prop ?action-designator (:to ?location))
             (not (equal ?location :move)))
        (and (desig-prop ?action-designator (:to :move))
             (desig-prop ?action-designator (:both :arms))
             (desig-prop ?action-designator (:left ?_))
             (desig-prop ?action-designator (:right ?_)))))

  (<- (action-desig ?action-designator (move-arm ?pose ?left-or-right))
    (desig-prop ?action-designator (:type :moving))
    (desig-prop ?action-designator (:arm ?left-or-right))
    (desig-prop ?action-designator (:goal ?location))
    (-> (lisp-type ?location designator)
        (desig-location-prop ?location ?pose)
        (or (cram-tf:pose ?pose ?location)
            (equal ?location ?pose))))

  (<- (action-desig ?action-designator (move-arm ?pose ?left-or-right))
    (desig-prop ?action-designator (:to :move))
    (desig-prop ?action-designator (?left-or-right :arm))
    (desig-prop ?action-designator (:to ?location))
    (not (equal ?location :move))
    (-> (lisp-type ?location designator)
        (desig-location-prop ?location ?pose)
        (or (cram-tf:pose ?pose ?location)
            (equal ?location ?pose))))

  (<- (action-desig ?action-designator (move-arm ?poses :both))
    (desig-prop ?action-designator (:to :move))
    (desig-prop ?action-designator (:both :arms))
    (desig-prop ?action-designator (:left ?left-location))
    (desig-prop ?action-designator (:right ?right-location))
    (and (-> (lisp-type ?left-location designator)
             (desig-location-prop ?left-location ?left-pose)
             (or (cram-tf:pose ?left-pose ?left-location)
                 (equal ?left-location ?left-pose)))
         (-> (lisp-type ?right-location designator)
             (desig-location-prop ?right-location ?right-pose)
             (or (cram-tf:pose ?right-pose ?right-location)
                 (equal ?right-location ?right-pose)))
         (equal ?poses (?left-pose ?right-pose))))

  (<- (available-process-module pr2-arms-pm)
    (not (projection-running ?_))))


;; (def-fact-group pr2-manipulation-actions (action-desig)
;;   ;; On the PR2 we don't need an open pose
;;   (<- (action-desig ?desig (noop ?desig))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:pose :open)))

;;   (<- (action-desig ?desig (park-object ?obj ?grasp-assignments))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :park))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (current-designator ?obj ?current-obj)
;;     (object->grasp-assignments ?current-obj ?grasp-assignments))

;;   (<- (action-desig ?desig (park-arms ?arms))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :park))
;;     (free-arms ?arms))

;;   (<- (action-desig ?desig (lift nil nil ?distance))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :lift))
;;     (desig-prop ?desig (:obj nil))
;;     (-> (desig-prop ?desig (:distance ?distance))
;;         (true)
;;         (== ?distance 0.1)))

;;   (<- (action-desig ?desig (lift ?current-obj ?grasp-assignments ?distance))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :lift))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (not (equal ?obj nil))
;;     (current-designator ?obj ?current-obj)
;;     (object->grasp-assignments ?current-obj ?grasp-assignments)
;;     (-> (desig-prop ?desig (:distance ?distance))
;;         (true)
;;         (== ?distance 0.02)))

;;   (<- (action-desig ?desig (park ?arms ?obj ?obstacles))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :carry))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (current-designator ?obj ?current-obj)
;;     (holding-arms ?current-obj ?arms)
;;     (obstacles ?desig ?obstacles))

;;   (<- (action-desig ?desig (grasp ?desig ?current-obj))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :grasp))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (newest-effective-designator ?obj ?current-obj))

;;   (<- (action-desig ?desig (shove-into ?current-obj ?target-pose))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :shove-into))
;;     (desig-prop ?desig (:obj ?obj))
;;     (current-designator ?obj ?current-obj)
;;     (desig-prop ?desig (:pose ?target-pose)))

;;   (<- (action-desig ?desig (pull-open ?semantic-handle))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :pull-open))
;;     (desig-prop ?desig (:handle ?semantic-handle)))

;;   (<- (action-desig ?desig (open-container ?arm ?loc ?degree))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :open))
;;     (desig-prop ?desig (:location ?loc))
;;     (desig-prop ?desig (:degree ?degree))
;;     (free-arm ?arm))

;;   (<- (action-desig ?desig (put-down ?current-obj ?loc ?grasp-assignments))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :put-down))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (desig-prop ?desig (:at ?loc))
;;     (current-designator ?obj ?current-obj)
;;     (object->grasp-assignments ?current-obj ?grasp-assignments))

;;   (<- (action-desig ?desig (pull ?current-obj ?arms ?direction ?distance ?obstacles))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :pull))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (desig-prop ?desig (:distance ?distance))
;;     (desig-prop ?desig (:direction ?direction))
;;     (current-designator ?obj ?current-obj)
;;     (fail) ;; This predicate needs to be refactored
;;     (grasped-object-part ?obj ?grasped)
;;     (holding-arms ?current-obj ?arms)
;;     (obstacles ?desig ?obstacles))

;;   (<- (action-desig ?desig (push ?current-obj ?arms ?direction ?distance ?obstacles))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :push))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (desig-prop ?desig (:distance ?distance))
;;     (desig-prop ?desig (:direction ?direction))
;;     (current-designator ?obj ?current-obj)
;;     (holding-arms ?current-obj ?arms)
;;     (obstacles ?desig ?obstacles)))

;; (def-fact-group manipulation-designators (action-desig-projection)
;;   (<- (action-desig-projection
;;        ?desig (execute-container-opened ?desig ?obj ?distance))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :open))
;;     (desig-prop ?desig (:handle ?obj))
;;     (desig-prop ?obj (:name ?handle-name))
;;     (-> (desig-prop ?desig (:side ?side))
;;         (== ?sides (?side))
;;         (true))
;;     (available-arms ?obj ?sides)
;;     (lisp-fun get-opening-distance ?handle-name ?distance))

;;   (<- (action-desig-projection ?desig (execute-container-closed ?desig ?obj))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :close))
;;     (desig-prop ?desig (:handle ?obj))
;;     (-> (desig-prop ?desig (:side ?side))
;;         (== ?sides (?side))
;;         (true))
;;     (available-arms ?obj ?sides))

;;   (<- (action-desig-projection ?desig (execute-lift ?desig))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :lift))
;;     (or (desig-prop ?desig (:obj ?_))
;;         (desig-prop ?desig (:object ?_))))

;;   (<- (action-desig-projection ?desig (execute-park ?sides ?objects-in-hand))
;;     (trajectory-desig? ?desig)
;;     (or
;;      (desig-prop ?desig (:to :park))
;;      (desig-prop ?desig (:to :carry)))
;;     (robot ?robot)
;;     (-> (desig-prop ?desig (:side ?side))
;;         (== ?sides ?side)
;;         (findall ?side (arm ?robot ?side) ?sides))
;;     (findall (?side ?obj ?link)
;;              (and
;;               (cram-plan-occasions-events:object-in-hand ?obj ?side) 
;;               (end-effector-link ?robot ?side ?link))
;;              ?objects-in-hand))

;;   (<- (action-desig-projection ?desig (execute-grasp ?desig ?obj))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :grasp))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj))))

;;   (<- (action-desig-projection ?desig (execute-put-down ?desig ?obj))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :put-down))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (desig-prop ?desig (:at ?_))))

