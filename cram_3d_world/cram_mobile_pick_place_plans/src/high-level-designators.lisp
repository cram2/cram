;;;
;;; Copyright (c) 2021, Amar Fayaz <amar@uni-bremen.de>
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

(def-fact-group pick-and-place-high-level-plans (desig:action-grounding)

  (<- (desig:action-grounding ?action-designator (go-with-target-retries
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :going))
    (spec:property ?action-designator (:target ?some-location-designator))
    (once (or (spec:property ?action-designator (:speed ?speed))
              (equal ?speed nil)))
    (desig:current-designator ?some-location-designator ?location-designator)
    (member ?pose-stamped ?poses)
    (desig:designator :action ((:type :going)
                               (:target ?location-designator)
                               (:speed ?speed))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (pick-up-with-grasp-retries
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :picking-up-with-grasp-retries))
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)

    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (man-int:robot-free-hand ?robot ?arm))

    (-> (spec:property ?action-designator (:grasps ?grasps))
        (true)
        (and (spec:property ?current-object-desig (:type ?object-type))
             (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
             (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)))
    (desig:designator :action ((:type :pick-up-with-grasp-retries)
                               (:object ?current-object-desig)
                               (:arm ?arm)
                               (:grasps ?grasps))
                      ?resolved-action-designator))


    (<- (desig:action-grounding ?action-designator (pick-up-with-arm-retries
                                                  ?resolved-action-designator))
    (or (spec:property ?action-designator (:type :picking-up))
        (spec:property ?action-designator (:type :picking-up-with-arm-retries)))
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)

    (-> (spec:property ?action-designator (:arms ?arms))
        (true)
        (setof ?arm (man-int:robot-free-hand ?_ ?arm) ?arms))

    (-> (spec:property ?action-designator (:grasps ?grasps))
        (true)
        (and (spec:property ?current-object-desig (:type ?object-type))
             (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
             (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)))

    (desig:designator :action ((:type :picking-up)
                               (:object ?current-object-desig)
                               (:arms ?arms)
                               (:grasps ?grasps))
                      ?resolved-action-designator)))  
