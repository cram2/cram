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

(in-package :pp-plans)

(def-fact-group pick-and-place-atomic-actions (desig:action-grounding)

  (<- (desig:action-grounding ?action-designator (go-to-target ?location-designator))
    (spec:property ?action-designator (:type :going))
    (spec:property ?action-designator (:target ?location-designator)))

  (<- (desig:action-grounding ?action-designator (perceive ?object-designator))
    (spec:property ?action-designator (:type :detecting))
      (spec:property ?action-designator (:object ?object-designator)))

  (<- (desig:action-grounding ?action-designator (move-arms-in-sequence ?left-poses ?right-poses))
    (or (spec:property ?action-designator (:type :reaching))
        (spec:property ?action-designator (:type :retracting)))
    (once (or (spec:property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (spec:property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil))))

  (<- (desig:action-grounding ?action-designator (move-arms-in-sequence
                                                  ?left-poses ?right-poses :allow-hand))
    (or (spec:property ?action-designator (:type :lifting)))
    (once (or (spec:property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (spec:property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil))))

  (<- (desig:action-grounding ?action-designator (move-arms-in-sequence
                                                  ?left-poses ?right-poses :allow-all))
    (or (spec:property ?action-designator (:type :pulling))
        (spec:property ?action-designator (:type :pushing))
        (spec:property ?action-designator (:type :putting))
        (spec:property ?action-designator (:type :grasping)))
    (once (or (spec:property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (spec:property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil))))

  (<- (desig:action-grounding ?action-designator (release ?left-or-right-or-both))
    (spec:property ?action-designator (:type :releasing))
    (spec:property ?action-designator (:gripper ?left-or-right-or-both)))

  (<- (desig:action-grounding ?action-designator (grip ?left-or-right-or-both ?object-grip-effort))
    (spec:property ?action-designator (:type :gripping))
    (spec:property ?action-designator (:gripper ?left-or-right-or-both))
    (once (or (spec:property ?action-designator (:effort ?object-grip-effort))
              (equal ?object-grip-effort nil))))

  (<- (desig:action-grounding ?action-designator (open-or-close-gripper ?left-or-right-or-both
                                                                        ?action-type))
    (or (spec:property ?action-designator (:type :closing))
        (spec:property ?action-designator (:type :opening)))
    (spec:property ?action-designator (:type ?action-type))
    (spec:property ?action-designator (:gripper ?left-or-right-or-both)))

  (<- (desig:action-grounding ?action-designator (set-gripper-to-position
                                                  ?left-or-right-or-both ?position))
    (spec:property ?action-designator (:type :setting-gripper))
    (spec:property ?action-designator (:gripper ?left-or-right-or-both))
    (spec:property ?action-designator (:position ?position)))

  (<- (desig:action-grounding ?action-designator (look-at :target ?location-designator))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:target ?location-designator))
    (-> (spec:property ?action-designator (:camera ?camera))
        (equal ?camera :head)
        (true)))
  (<- (desig:action-grounding ?action-designator (look-at :frame ?frame))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:frame ?frame))
    (-> (spec:property ?action-designator (:camera ?camera))
        (equal ?camera :head)
        (true)))
  (<- (desig:action-grounding ?action-designator (look-at :direction ?direction))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:direction ?direction))
    (-> (spec:property ?action-designator (:camera ?camera))
        (equal ?camera :head)
        (true)))
  (<- (desig:action-grounding ?action-designator (look-at :object ?object-designator))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:object ?object-designator))
    (-> (spec:property ?action-designator (:camera ?camera))
        (equal ?camera :head)
        (true)))

)
