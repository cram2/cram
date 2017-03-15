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
    (or (and (desig-prop ?action-designator (:type :going-motion))
             (desig-prop ?action-designator (:destination ?_)))
        (and (desig-prop ?action-designator (:to :go-motion))
             (desig-prop ?action-designator (:to ?where))
             (not (equal ?where :go-motion)))))

  (<- (action-desig ?action-designator (drive ?pose))
    (desig-prop ?action-designator (:type :going-motion))
    (desig-prop ?action-designator (:destination ?where))
    (-> (lisp-type ?where designator)
        (desig-location-prop ?where ?pose)
        (or (cram-tf:pose ?pose ?where)
            (equal ?where ?pose))))

  (<- (action-desig ?action-designator (drive ?pose))
    (desig-prop ?action-designator (:to :go-motion))
    (desig-prop ?action-designator (:to ?where))
    (not (equal ?where :go-motion))
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
    (or (and (desig-prop ?action-designator (:type :looking-motion))
             (or (desig-prop ?action-designator (:pose ?_))
                 (desig-prop ?action-designator (:frame ?_))
                 (desig-prop ?action-designator (:location ?_))
                 (desig-prop ?action-designator (:object ?_))))
        (and (desig-prop ?action-designator (:to :look-motion))
             (desig-prop ?action-designator (:at ?_)))))

  (<- (action-desig ?action-designator (look-at :point ?pose))
    (desig-prop ?action-designator (:type :looking-motion))
    (or (desig-prop ?action-designator (:pose ?pose))
        (and (or (desig-prop ?action-designator (:location ?obj-or-loc))
                 (desig-prop ?action-designator (:object ?obj-or-loc)))
             (desig-location-prop ?obj-or-loc ?pose))))

  (<- (action-desig ?action-designator (look-at :point ?pose))
    (desig-prop ?action-designator (:to :look-motion))
    (desig-prop ?action-designator (:at ?where))
    (-> (lisp-type ?where designator)
        (desig-location-prop ?where ?pose)
        (or (cram-tf:pose ?pose ?where)
            (equal ?where ?pose))))

  (<- (action-desig ?action-designator (look-at :frame ?frame))
    (desig-prop ?action-designator (:type :looking-motion))
    (desig-prop ?action-designator (:frame ?frame)))

  (<- (available-process-module pr2-ptu-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-perception-actions (action-desig
                                        matching-process-module
                                        available-process-module)

  (<- (matching-process-module ?action-designator pr2-perception-pm)
    (or (desig-prop ?action-designator (:type :detecting-motion))
        (desig-prop ?action-designator (:to :detect-motion)))
    (or (desig-prop ?action-designator (:object ?object-designator))
        (desig-prop ?action-designator (:objects ?object-designator)))
    ;; (current-designator ?object-designator ?current-object-designator)
    ;; (desig-prop ?current-object-designator (:type ?object-type))
    )

  (<- (action-desig ?action-designator (detect ?object-properties ?quantifier))
    (or (desig-prop ?action-designator (:type :detecting-motion))
        (desig-prop ?action-designator (:to :detect-motion)))
    (or (and (desig-prop ?action-designator (:object ?object-designator))
             (equal ?quantifier :an))
        (and (desig-prop ?action-designator (:objects ?object-designator))
             (equal ?quantifier :all)))
    (-> (lisp-type ?object-designator object-designator)
        (and (current-designator ?object-designator ?current-object-designator)
             (desig-description ?current-object-designator ?object-properties))
        (prolog:equal ?object-properties ((:type ?object-designator)))))

  (<- (available-process-module pr2-perception-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-gripper-actions (action-desig
                                     matching-process-module
                                     available-process-module)

  (<- (matching-process-module ?action-designator pr2-grippers-pm)
    (or (and (or (desig-prop ?action-designator (:type :gripping-motion))
                 (desig-prop ?action-designator (:to :grip-motion)))
             (or (desig-prop ?action-designator (:gripper ?which-gripper))
                 (desig-prop ?action-designator (:with ?which-gripper))))
        (and (or (desig-prop ?action-designator (:type :opening-motion))
                 (desig-prop ?action-designator (:type :closing-motion)))
             (desig-prop ?action-designator (:gripper ?_)))
        (and (or (desig-prop ?action-designator (:to :open-motion))
                 (desig-prop ?action-designator (:to :close-motion)))
             (desig-prop ?action-designator (?_ :gripper)))))

  (<- (action-desig ?action-designator (gripper-action :open ?which-gripper))
    (desig-prop ?action-designator (:type :opening-motion))
    (desig-prop ?action-designator (:gripper ?which-gripper)))

  (<- (action-desig ?action-designator (gripper-action :open ?which-gripper))
    (desig-prop ?action-designator (:to :open-motion))
    (desig-prop ?action-designator (?which-gripper :gripper)))

  (<- (action-desig ?action-designator (gripper-action :close ?which-gripper))
    (desig-prop ?action-designator (:type :closing-motion))
    (desig-prop ?action-designator (:gripper ?which-gripper)))

  (<- (action-desig ?action-designator (gripper-action :close ?which-gripper))
    (desig-prop ?action-designator (:to :close-motion))
    (desig-prop ?action-designator (?which-gripper :gripper)))

  (<- (action-desig ?action-designator (gripper-action :grip ?which-gripper ?maximum-effort))
    (or (desig-prop ?action-designator (:type :gripping-motion))
        (desig-prop ?action-designator (:to :grip-motion)))
    (or (desig-prop ?action-designator (:gripper ?which-gripper))
        (desig-prop ?action-designator (:with ?which-gripper)))
    (once (or (desig-prop ?action-designator (:effort ?maximum-effort))
              (equal ?maximum-effort nil))))

  (<- (available-process-module pr2-grippers-pm)
    (not (projection-running ?_))))


(def-fact-group pr2-arm-actions (action-desig
                                 matching-process-module
                                 available-process-module)

  (<- (matching-process-module ?action-designator pr2-arms-pm)
    (or (desig-prop ?action-designator (:to :move-arm-motion))
        (desig-prop ?action-designator (:type :moving-arm-motion))
        (desig-prop ?action-designator (:to :move-joints-motion))
        (desig-prop ?action-designator (:type :moving-joints-motion)))
    (or (desig-prop ?action-designator (:left ?locations))
        (desig-prop ?action-designator (:right ?locations))))

  (<- (action-desig ?action-designator (move-arm ?pose-left ?pose-right))
    (or (desig-prop ?action-designator (:to :move-arm-motion))
        (desig-prop ?action-designator (:type :moving-arm-motion)))
    (-> (desig-prop ?action-designator (:left ?left-location))
        (%parse-poses ?left-location ?pose-left)
        (equal ?pose-left nil))
    (-> (desig-prop ?action-designator (:right ?right-location))
        (%parse-poses ?right-location ?pose-right)
        (equal ?pose-right nil)))

  (<- (action-desig ?action-designator (move-joints ?left-configuration ?right-configuration))
    (or (desig-prop ?action-designator (:to :move-joints-motion))
        (desig-prop ?action-designator (:type :moving-joints-motion)))
    (-> (desig-prop ?action-designator (:left ?left-configuration))
        (true)
        (equal ?left-configuration nil))
    (-> (desig-prop ?action-designator (:right ?right-configuration))
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

