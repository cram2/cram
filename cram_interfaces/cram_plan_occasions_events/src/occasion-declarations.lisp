;;;
;;; Copyright (c) 2015, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-plan-occasions-events)

(def-fact-group occasions (object-in-hand
                           object-at-location robot-at-location
                           torso-at arms-positioned-at tool-frames-at
                           looking-at
                           container-state)

  (<- (object-in-hand ?object ?hand ?grasp ?link)
    (fail))
  (<- (object-in-hand ?object ?hand ?grasp)
    (fail))
  (<- (object-in-hand ?object ?hand)
    (fail))
  (<- (object-in-hand ?object)
    (fail))

  (<- (object-at-location ?object-designator ?location-designator)
    (fail))
  (<- (robot-at-location ?object-designator ?location-designator)
    (fail))

  (<- (torso-at ?joint-state)
    (fail))
  (<- (torso-at ?joint-state ?delta)
    (fail))

  (<- (arms-positioned-at ?left-configuration ?right-configuration)
    (fail))
  (<- (arms-positioned-at ?left-configuration ?right-configuration ?delta)
    (fail))

  (<- (tool-frames-at ?left-poses ?right-poses)
    (fail))
  (<- (tool-frames-at ?left-poses ?right-poses ?delta-position ?delta-rotation)
    (fail))

  (<- (looking-at ?location-or-object-or-frame-or-direction-or-pose)
    (fail))
  (<- (looking-at ?location-or-object-or-frame-or-direction-or-pose ?delta)
    (fail))

  (<- (container-state ?container-object-designator ?joint-state)
    (fail)))
