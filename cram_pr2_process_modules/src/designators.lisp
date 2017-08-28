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

(def-fact-group pr2-navigation-actions (motion-grounding
                                        matching-process-module
                                        available-process-module)

  (<- (matching-process-module ?action-designator pr2-base-pm)
    (or (and (desig-prop ?action-designator (:type :going))
             (desig-prop ?action-designator (:destination ?_)))
        (and (desig-prop ?action-designator (:to :go))
             (desig-prop ?action-designator (:to ?where))
             (not (equal ?where :go)))))

  (<- (motion-grounding ?motion-designator (drive ?pose))
    (desig-prop ?motion-designator (:type :going))
    (desig-prop ?motion-designator (:destination ?where))
    (-> (lisp-type ?where designator)
        (desig-location-prop ?where ?pose)
        (or (cram-tf:pose ?pose ?where)
            (equal ?where ?pose))))

  (<- (motion-grounding ?motion-designator (drive ?pose))
    (desig-prop ?motion-designator (:to :go))
    (desig-prop ?motion-designator (:to ?where))
    (not (equal ?where :go))
    (-> (lisp-type ?where designator)
        (desig-location-prop ?where ?pose)
        (or (cram-tf:pose ?pose ?where)
            (equal ?where ?pose))))

  (<- (available-process-module pr2-base-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-ptu-actions (motion-grounding
                                 matching-process-module
                                 available-process-module)

  (<- (matching-process-module ?motion-designator pr2-ptu-pm)
    (or (and (desig-prop ?motion-designator (:type :looking))
             (or (desig-prop ?motion-designator (:pose ?_))
                 (desig-prop ?motion-designator (:frame ?_))
                 (desig-prop ?motion-designator (:location ?_))
                 (desig-prop ?motion-designator (:object ?_))))
        (and (desig-prop ?motion-designator (:to :look))
             (desig-prop ?motion-designator (:at ?_)))))

  (<- (motion-grounding ?motion-designator (look-at :point ?pose))
    (desig-prop ?motion-designator (:type :looking))
    (or (desig-prop ?motion-designator (:pose ?pose))
        (and (or (desig-prop ?motion-designator (:location ?obj-or-loc))
                 (desig-prop ?motion-designator (:object ?obj-or-loc)))
             (desig-location-prop ?obj-or-loc ?pose))))

  (<- (motion-grounding ?motion-designator (look-at :point ?pose))
    (desig-prop ?motion-designator (:to :look))
    (desig-prop ?motion-designator (:at ?where))
    (-> (lisp-type ?where designator)
        (desig-location-prop ?where ?pose)
        (or (cram-tf:pose ?pose ?where)
            (equal ?where ?pose))))

  (<- (motion-grounding ?motion-designator (look-at :frame ?frame))
    (desig-prop ?motion-designator (:type :looking))
    (desig-prop ?motion-designator (:frame ?frame)))

  (<- (available-process-module pr2-ptu-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-perception-actions (motion-grounding
                                        matching-process-module
                                        available-process-module)

  (<- (matching-process-module ?motion-designator pr2-perception-pm)
    (or (desig-prop ?motion-designator (:type :detecting))
        (desig-prop ?motion-designator (:to :detect)))
    (or (desig-prop ?motion-designator (:object ?object-designator))
        (desig-prop ?motion-designator (:objects ?object-designator)))
    ;; (current-designator ?object-designator ?current-object-designator)
    ;; (desig-prop ?current-object-designator (:type ?object-type))
    )

  (<- (motion-grounding ?motion-designator (detect ?object-properties ?quantifier))
    (or (desig-prop ?motion-designator (:type :detecting))
        (desig-prop ?motion-designator (:to :detect)))
    (or (and (desig-prop ?motion-designator (:object ?object-designator))
             (equal ?quantifier :an))
        (and (desig-prop ?motion-designator (:objects ?object-designator))
             (equal ?quantifier :all)))
    (-> (lisp-type ?object-designator object-designator)
        (and (current-designator ?object-designator ?current-object-designator)
             (desig-description ?current-object-designator ?object-properties))
        (prolog:equal ?object-properties ((:type ?object-designator)))))

  (<- (available-process-module pr2-perception-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-gripper-actions (motion-grounding
                                     matching-process-module
                                     available-process-module)

  (<- (matching-process-module ?motion-designator pr2-grippers-pm)
    (or (and (or (desig-prop ?motion-designator (:type :gripping))
                 (desig-prop ?motion-designator (:to :grip)))
             (or (desig-prop ?motion-designator (:gripper ?which-gripper))
                 (desig-prop ?motion-designator (:with ?which-gripper))))
        (and (or (desig-prop ?motion-designator (:type :opening))
                 (desig-prop ?motion-designator (:type :closing)))
             (desig-prop ?motion-designator (:gripper ?_)))
        (and (or (desig-prop ?motion-designator (:to :open))
                 (desig-prop ?motion-designator (:to :close)))
             (desig-prop ?motion-designator (?_ :gripper)))))

  (<- (motion-grounding ?motion-designator (gripper-action :open ?which-gripper))
    (desig-prop ?motion-designator (:type :opening))
    (desig-prop ?motion-designator (:gripper ?which-gripper)))

  (<- (motion-grounding ?motion-designator (gripper-action :open ?which-gripper))
    (desig-prop ?motion-designator (:to :open))
    (desig-prop ?motion-designator (?which-gripper :gripper)))

  (<- (motion-grounding ?motion-designator (gripper-action :close ?which-gripper))
    (desig-prop ?motion-designator (:type :closing))
    (desig-prop ?motion-designator (:gripper ?which-gripper)))

  (<- (motion-grounding ?motion-designator (gripper-action :close ?which-gripper))
    (desig-prop ?motion-designator (:to :close))
    (desig-prop ?motion-designator (?which-gripper :gripper)))

  (<- (motion-grounding ?motion-designator (gripper-action :grip ?which-gripper ?maximum-effort))
    (or (desig-prop ?motion-designator (:type :gripping))
        (desig-prop ?motion-designator (:to :grip)))
    (or (desig-prop ?motion-designator (:gripper ?which-gripper))
        (desig-prop ?motion-designator (:with ?which-gripper)))
    (once (or (desig-prop ?motion-designator (:effort ?maximum-effort))
              (equal ?maximum-effort nil))))

  (<- (available-process-module pr2-grippers-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-arm-actions (motion-grounding
                                 matching-process-module
                                 available-process-module)

  (<- (matching-process-module ?motion-designator pr2-arms-pm)
    (or (desig-prop ?motion-designator (:to :move-arm))
        (desig-prop ?motion-designator (:type :moving-arm))
        (desig-prop ?motion-designator (:to :move-arm-joints))
        (desig-prop ?motion-designator (:type :moving-arm-joints)))
    (or (desig-prop ?motion-designator (:left ?locations))
        (desig-prop ?motion-designator (:right ?locations))))

  (<- (motion-grounding ?motion-designator (move-arm ?pose-left ?pose-right))
    (or (desig-prop ?motion-designator (:to :move-arm))
        (desig-prop ?motion-designator (:type :moving-arm)))
    (-> (desig-prop ?motion-designator (:left ?left-location))
        (%parse-poses ?left-location ?pose-left)
        (equal ?pose-left nil))
    (-> (desig-prop ?motion-designator (:right ?right-location))
        (%parse-poses ?right-location ?pose-right)
        (equal ?pose-right nil)))

  (<- (motion-grounding ?motion-designator (move-joints ?left-configuration ?right-configuration))
    (or (desig-prop ?motion-designator (:to :move-arm-joints))
        (desig-prop ?motion-designator (:type :moving-arm-joints)))
    (-> (desig-prop ?motion-designator (:left ?left-configuration))
        (true)
        (equal ?left-configuration nil))
    (-> (desig-prop ?motion-designator (:right ?right-configuration))
        (true)
        (equal ?right-configuration nil)))

  (<- (%parse-poses ?pose-description ?parsed-pose)
    (-> (lisp-type ?pose-description designator)
        (desig-location-prop ?pose-description ?parsed-pose)
        (once (or (and (cram-tf:pose ?parsed-pose ?pose-description))
                  (and (bagof ?parsed-single-pose
                              (and (member ?single-pose-description ?pose-description)
                                   (%parse-poses ?single-pose-description ?parsed-single-pose))
                              ?parsed-single-poses-lazy)
                       (lisp-fun cut:force-ll ?parsed-single-poses-lazy ?parsed-pose))
                  (equal ?pose-description ?parsed-pose)))))

  (<- (available-process-module pr2-arms-pm)
    (not (projection-running ?_))))


;; (def-fact-group pr2-manipulation-actions (motion-desig)
;;   ;; On the PR2 we don't need an open pose
;;   (<- (motion-desig ?desig (noop ?desig))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:pose :open)))

;;   (<- (motion-desig ?desig (park-object ?obj ?grasp-assignments))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :park))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (current-designator ?obj ?current-obj)
;;     (object->grasp-assignments ?current-obj ?grasp-assignments))

;;   (<- (motion-desig ?desig (park-arms ?arms))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :park))
;;     (free-arms ?arms))

;;   (<- (motion-desig ?desig (lift nil nil ?distance))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :lift))
;;     (desig-prop ?desig (:obj nil))
;;     (-> (desig-prop ?desig (:distance ?distance))
;;         (true)
;;         (== ?distance 0.1)))

;;   (<- (motion-desig ?desig (lift ?current-obj ?grasp-assignments ?distance))
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

;;   (<- (motion-desig ?desig (park ?arms ?obj ?obstacles))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :carry))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (current-designator ?obj ?current-obj)
;;     (holding-arms ?current-obj ?arms)
;;     (obstacles ?desig ?obstacles))

;;   (<- (motion-desig ?desig (grasp ?desig ?current-obj))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :grasp))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (newest-effective-designator ?obj ?current-obj))

;;   (<- (motion-desig ?desig (shove-into ?current-obj ?target-pose))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :shove-into))
;;     (desig-prop ?desig (:obj ?obj))
;;     (current-designator ?obj ?current-obj)
;;     (desig-prop ?desig (:pose ?target-pose)))

;;   (<- (motion-desig ?desig (pull-open ?semantic-handle))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :pull-open))
;;     (desig-prop ?desig (:handle ?semantic-handle)))

;;   (<- (motion-desig ?desig (open-container ?arm ?loc ?degree))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :open))
;;     (desig-prop ?desig (:location ?loc))
;;     (desig-prop ?desig (:degree ?degree))
;;     (free-arm ?arm))

;;   (<- (motion-desig ?desig (put-down ?current-obj ?loc ?grasp-assignments))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :put-down))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (desig-prop ?desig (:at ?loc))
;;     (current-designator ?obj ?current-obj)
;;     (object->grasp-assignments ?current-obj ?grasp-assignments))

;;   (<- (motion-desig ?desig (pull ?current-obj ?arms ?direction ?distance ?obstacles))
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

;;   (<- (motion-desig ?desig (push ?current-obj ?arms ?direction ?distance ?obstacles))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :push))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (desig-prop ?desig (:distance ?distance))
;;     (desig-prop ?desig (:direction ?direction))
;;     (current-designator ?obj ?current-obj)
;;     (holding-arms ?current-obj ?arms)
;;     (obstacles ?desig ?obstacles)))

;; (def-fact-group manipulation-designators (motion-desig-projection)
;;   (<- (motion-desig-projection
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

;;   (<- (motion-desig-projection ?desig (execute-container-closed ?desig ?obj))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :close))
;;     (desig-prop ?desig (:handle ?obj))
;;     (-> (desig-prop ?desig (:side ?side))
;;         (== ?sides (?side))
;;         (true))
;;     (available-arms ?obj ?sides))

;;   (<- (motion-desig-projection ?desig (execute-lift ?desig))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :lift))
;;     (or (desig-prop ?desig (:obj ?_))
;;         (desig-prop ?desig (:object ?_))))

;;   (<- (motion-desig-projection ?desig (execute-park ?sides ?objects-in-hand))
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

;;   (<- (motion-desig-projection ?desig (execute-grasp ?desig ?obj))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :grasp))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj))))

;;   (<- (motion-desig-projection ?desig (execute-put-down ?desig ?obj))
;;     (trajectory-desig? ?desig)
;;     (desig-prop ?desig (:to :put-down))
;;     (or (desig-prop ?desig (:obj ?obj))
;;         (desig-prop ?desig (:object ?obj)))
;;     (desig-prop ?desig (:at ?_))))

