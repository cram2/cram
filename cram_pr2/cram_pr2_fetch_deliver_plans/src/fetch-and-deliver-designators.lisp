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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
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

(in-package :pr2-fd-plans)

(def-fact-group fetch-and-deliver-designators (desig:action-grounding)

  (<- (desig:action-grounding ?action-designator (go-without-collisions ?location-designator))
    (spec:property ?action-designator (:type :navigating))
    (spec:property ?action-designator (:location ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator))

  (<- (desig:action-grounding ?action-designator (search-for-object
                                                  ?object-designator ?location-designator))
    (spec:property ?action-designator (:type :searching))
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    (spec:property ?action-designator (:location ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator))

  (<- (desig:action-grounding ?action-designator (fetch ?object-designator ?arm
                                                        ?location-designator
                                                        ?pick-up-action-designator))
    (spec:property ?action-designator (:type :fetching))
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    (desig:desig-prop ;; spec:property
     ?action-designator (:robot-location ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    (desig:desig-prop ;; spec:property
     ?action-designator (:pick-up-action ?some-pick-up-action-designator))
    (desig:current-designator ?some-pick-up-action-designator ?pick-up-action-designator)
    (or (spec:property ?action-designator (:arm ?arm))
        (equal ?arm NIL)))

  (<- (desig:action-grounding ?action-designator (deliver ?object-designator ?location-designator
                                                          ?robot-location-designator
                                                          ?place-action-designator))
    (spec:property ?action-designator (:type :delivering))
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    (spec:property ?action-designator (:target ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    (desig:desig-prop ;; spec:property
     ?action-designator (:robot-location ?some-robot-location-designator))
    (desig:current-designator ?some-robot-location-designator ?robot-location-designator)
    (desig:desig-prop ;; spec:property
     ?action-designator (:place-action ?some-place-action-designator))
    (desig:current-designator ?some-place-action-designator ?place-action-designator))

  (<- (desig:action-grounding ?action-designator (transport
                                                  ?object-designator
                                                  ?fetching-location-designator
                                                  ?delivering-location-designator
                                                  ?arm))
    (spec:property ?action-designator (:type :transporting))
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    (spec:property ?action-designator (:location ?some-fetching-location-designator))
    (desig:current-designator ?some-fetching-location-designator ?fetching-location-designator)
    (spec:property ?action-designator (:target ?some-delivering-location-designator))
    (desig:current-designator ?some-delivering-location-designator ?delivering-location-designator)
    (or (spec:property ?action-designator (:arm ?arm))
        (equal ?arm NIL))))
