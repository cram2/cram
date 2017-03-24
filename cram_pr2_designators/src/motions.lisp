;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-pr2-designators)

(def-fact-group pr2-navigation-motions (motion-grounding)

  (<- (motion-grounding ?motion-designator (drive ?pose))
    (desig-prop ?motion-designator (:type :going))
    (desig-prop ?motion-designator (:pose ?pose))
    (lisp-type ?pose cl-transforms-stamped:pose-stamped))

  (<- (motion-grounding ?motion-designator (drive ?pose))
    (desig-prop ?motion-designator (:type :going))
    (or (and (desig-prop ?motion-designator (:pose ?pose)))
        (and (or (desig-prop ?motion-designator (:destination ?location))
                 (desig-prop ?motion-designator (:location ?location)))
             ))
    (or 
        (desig-prop ?motion-designator (:to :go)))
    (or (desig-prop ?motion-designator (:destination ?location))
        (and (desig-prop ?motion-designator (:to ?location))
             (not (equal ?location :go))))
    (cram-tf:location-pose ?where ?pose)))


(def-fact-group pr2-ptu-motions (motion-grounding)

  (<- (motion-grounding ?motion-designator (look-at :point ?pose))
    (or (desig-prop ?motion-designator (:to :look))
        (desig-prop ?motion-designator (:type :looking)))
    (or(desig-prop ?motion-designator (:at ?location)))
    (or (desig-prop ?motion-designator (:pose ?pose))
        (and 
             (not (equal ?location :look)))
        (and (or (desig-prop ?motion-designator (:location ?obj-or-loc))
                 (desig-prop ?motion-designator (:object ?obj-or-loc)))
             (desig-location-prop ?obj-or-loc ?pose))))

  (<- (motion-grounding ?motion-designator (look-at :point ?pose))
    
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
        (desig-prop ?motion-designator (:to :move-joints))
        (desig-prop ?motion-designator (:type :moving-joints)))
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
    (or (desig-prop ?motion-designator (:to :move-joints))
        (desig-prop ?motion-designator (:type :moving-joints)))
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
