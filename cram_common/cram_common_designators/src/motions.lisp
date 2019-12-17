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

(in-package :cram-common-designators)

(def-fact-group navigation-motions (motion-grounding)

  (<- (motion-grounding ?designator (move-base ?pose-stamped))
    (property ?designator (:type :going))
    (property ?designator (:pose ?pose-stamped))))


(def-fact-group torso-motions (motion-grounding)

  (<- (motion-grounding ?designator (move-torso ?joint-angle))
    (property ?designator (:type :moving-torso))
    (property ?designator (:joint-angle ?joint-angle))))


(def-fact-group ptu-motions (motion-grounding)

  (<- (motion-grounding ?designator (move-head ?pose-stamped ?configuration))
    (property ?designator (:type :looking))
    (-> (property ?designator (:pose ?pose-stamped))
        (true)
        (equal ?pose-stamped nil))
    (-> (property ?designator (:joint-states ?configuration))
        (true)
        (equal ?configuration nil))))


(def-fact-group perception-motions (motion-grounding)

  (<- (motion-grounding ?designator (detect ?current-object-designator))
    (property ?designator (:type :detecting))
    (property ?designator (:object ?object-designator))
    (current-designator ?object-designator ?current-object-designator))

  (<- (motion-grounding ?designator (inspect ?current-object-designator))
    (property ?designator (:type :inspecting))
    (property ?designator (:object ?object-designator))
    (current-designator ?object-designator ?current-object-designator)))


(def-fact-group gripper-motions (motion-grounding)

  (<- (motion-grounding ?designator (move-gripper-joint :open ?which-gripper))
    (property ?designator (:type :opening-gripper))
    (property ?designator (:gripper ?which-gripper)))

  (<- (motion-grounding ?designator (move-gripper-joint :close ?which-gripper))
    (property ?designator (:type :closing-gripper))
    (property ?designator (:gripper ?which-gripper)))

  (<- (motion-grounding ?designator (move-gripper-joint :grip ?which-gripper ?maximum-effort))
    (property ?designator (:type :gripping))
    (property ?designator (:gripper ?which-gripper))
    (once (or (property ?designator (:effort ?maximum-effort))
              (equal ?maximum-effort nil))))

  (<- (desig:motion-grounding ?designator (move-gripper-joint ?position ?which-gripper NIL))
    (property ?designator (:type :moving-gripper-joint))
    (property ?designator (:gripper ?which-gripper))
    (property ?designator (:joint-angle ?position))))


(def-fact-group arm-motions (motion-grounding)

  (<- (motion-grounding ?designator (move-tcp ?left-pose ?right-pose
                                              ?collision-mode
                                              ?collision-object-b ?collision-object-b-link
                                              ?collision-object-a
                                              ?move-base ?prefer-base
                                              ?align-planes-left ?align-planes-right))
    (property ?designator (:type :moving-tcp))
    (once (or (property ?designator (:left-pose ?left-pose))
              (equal ?left-pose nil)))
    (once (or (property ?designator (:right-pose ?right-pose))
              (equal ?right-pose nil)))
    (once (or (desig:desig-prop ?designator (:collision-mode ?collision-mode))
              (equal ?collision-mode nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-b ?collision-object-b))
              (equal ?collision-object-b nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-b-link
                                             ?collision-object-b-link))
              (equal ?collision-object-b-link nil)))
    (once (or (desig:desig-prop ?designator (:collision-object-a ?collision-object-a))
              (equal ?collision-object-a nil)))
    (once (or (desig:desig-prop ?designator (:move-base ?move-base))
              (equal ?move-base nil)))
    (once (or (desig:desig-prop ?designator (:prefer-base ?prefer-base))
              (equal ?prefer-base nil)))
    (once (or (desig:desig-prop ?designator (:align-planes-left ?align-planes-left))
              (equal ?align-planes-left nil)))
    (once (or (desig:desig-prop ?designator (:align-planes-right ?align-planes-right))
              (equal ?align-planes-right nil))))

  (<- (motion-grounding ?designator (move-joints ?left-config ?right-config))
    (property ?designator (:type :moving-arm-joints))
    (once (or (property ?designator (:left-joint-states ?left-config))
              (equal ?left-config nil)))
    (once (or (property ?designator (:right-joint-states ?right-config))
              (equal ?right-config nil)))))

(def-fact-group world-state-detecting (motion-grounding)

  (<- (motion-grounding ?designator (world-state-detect ?object-designator))
    (property ?designator (:type :world-state-detecting))
    (property ?designator (:object ?object))
    (desig:current-designator ?object ?object-designator)
    (or (and (property ?object-designator (:pose ?_))
             (property ?object-designator (:type ?_))
             (property ?object-designator (:name ?_)))
        (property ?object-designator (:name ?_)))))
