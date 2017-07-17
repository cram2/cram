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

(def-fact-group boxy-atomic-actions (desig:action-grounding)

  (<- (desig:action-grounding ?action-designator (move-arms-in-sequence ?left-poses
                                                                        ?right-poses))
    (or (property ?action-designator (:type :reaching))
        (property ?action-designator (:type :retracting))
        (property ?action-designator (:type :lifting))
        (property ?action-designator (:type :putting)))
    (once (or (property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil))))

  (<- (desig:action-grounding ?action-designator (wiggle ?left-poses ?right-poses))
    (property ?action-designator (:type :pushing))
    (once (or (property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil))))

  (<- (desig:action-grounding ?action-designator (release ?left-or-right))
    (or (property ?action-designator (:type :releasing))
        (property ?action-designator (:type :opening)))
    (property ?action-designator (:gripper ?left-or-right)))

  (<- (desig:action-grounding ?action-designator (grip ?left-or-right ?object-grip-effort))
    (property ?action-designator (:type :gripping))
    (property ?action-designator (:arm ?left-or-right))
    (once (or (property ?action-designator (:effort ?object-grip-effort))
              (equal ?effort nil))))

  (<- (desig:action-grounding ?action-designator (close-gripper ?left-or-right))
    (property ?action-designator (:type :closing))
    (property ?action-designator (:gripper ?left-or-right)))

  (<- (desig:action-grounding ?action-designator (set-gripper-to-position ?left-or-right
                                                                          ?position))
    (property ?action-designator (:type :setting-gripper))
    (property ?action-designator (:gripper ?left-or-right))
    (property ?action-designator (:position ?position)))

  (<- (desig:action-grounding ?action-designator (perceive :detect ?object-designator))
    (property ?action-designator (:type :detecting))
    (property ?action-designator (:object ?object-designator)))
  (<- (desig:action-grounding ?action-designator (perceive :inspect ?object-designator))
    (property ?action-designator (:type :inspecting))
    (property ?action-designator (:object ?object-designator))))
