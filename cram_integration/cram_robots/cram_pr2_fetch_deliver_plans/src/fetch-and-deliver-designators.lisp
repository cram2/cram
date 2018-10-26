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

(defun calculate-pose-towards-target (look-pose-stamped robot-pose-stamped)
  "Given a `look-pose-stamped' and a `robot-pose-stamped' (both in fixed frame),
calculate the new robot-pose-stamped, which is rotated with an angle to point towards
the `look-pose-stamped'."
  (let* ((world->robot-transform
           (cram-tf:pose-stamped->transform-stamped robot-pose-stamped "robot"))
         (robot->world-transform
           (cl-transforms:transform-inv world->robot-transform))
         (world->look-pose-origin
           (cl-transforms:origin look-pose-stamped))
         (look-pose-in-robot-frame
           (cl-transforms:transform-point
            robot->world-transform
            world->look-pose-origin))
         (rotation-angle
           (atan
            (cl-transforms:y look-pose-in-robot-frame)
            (cl-transforms:x look-pose-in-robot-frame))))
    (cram-tf:rotate-pose robot-pose-stamped :z rotation-angle)))

(defun calculate-robot-navigation-goal-towards-target (target-designator)
  (calculate-pose-towards-target
   (desig:reference target-designator)
   (cram-tf:robot-current-pose)))

(def-fact-group fetch-and-deliver-designators (desig:action-grounding)

  (<- (desig:action-grounding ?action-designator (go-without-collisions ?location-designator))
    (spec:property ?action-designator (:type :navigating))
    (spec:property ?action-designator (:location ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator))

  (<- (desig:action-grounding ?action-designator (turn-towards ?location-designator
                                                               ?robot-location))
    (spec:property ?action-designator (:type :turning-towards))
    ;; target
    (spec:property ?action-designator (:target ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    ;; robot-location
    (lisp-fun calculate-robot-navigation-goal-towards-target ?location-designator
              ?robot-rotated-pose)
    (desig:designator :location ((:pose ?robot-rotated-pose)) ?robot-location))

  (<- (desig:action-grounding ?action-designator (manipulate-environment ?action-type
                                                                         ?object-designator
                                                                         ?arm
                                                                         ?distance
                                                                         ?robot-location))
    (or (spec:property ?action-designator (:type :accessing))
        (spec:property ?action-designator (:type :sealing)))
    (spec:property ?action-designator (:type ?action-type))
    ;; location
    (spec:property ?action-designator (:location ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    ;; object
    (desig:desig-prop ?location-designator (:in ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; arm
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (and (cram-robot-interfaces:robot ?robot)
             (cram-robot-interfaces:arm ?robot ?arm)
             (equal ?arm :left)))
    ;; distance
    (or (spec:property ?action-designator (:distance ?distance))
        (equal ?distance NIL))
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location ?some-robot-location))
                   (desig:current-designator ?robot-location ?robot-location))
              (desig:designator :location ((:reachable-for cram-pr2-description:pr2)
                                           (:arm ?arm)
                                           (:object ?object-designator))
                                ?robot-location))))

  (<- (desig:action-grounding ?action-designator (search-for-object
                                                  ?object-designator
                                                  ?location-designator
                                                  ?location-to-stand))
    (spec:property ?action-designator (:type :searching))
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; location
    (spec:property ?action-designator (:location ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location ?some-location-to-stand))
                   (desig:current-designator ?some-location-to-stand ?location-to-stand))
              (desig:designator :location ((:visible-for cram-pr2-description:pr2)
                                           (:location ?location-designator))
                                ?location-to-stand))))

  (<- (desig:action-grounding ?action-designator (fetch ?object-designator ?arm
                                                        ?robot-location-designator
                                                        ?pick-up-action-designator))
    (spec:property ?action-designator (:type :fetching))
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; arm
    (or (spec:property ?action-designator (:arm ?arm))
        (equal ?arm NIL))
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location ?some-location-designator))
                   (desig:current-designator ?some-location-designator ?robot-location-designator))
              (-> (spec:property ?action-designator (:arm ?arm))
                  (desig:designator :location ((:reachable-for cram-pr2-description:pr2)
                                               (:arm ?arm)
                                               (:object ?object-designator))
                                    ?robot-location-designator)
                  (desig:designator :location ((:reachable-for cram-pr2-description:pr2)
                                               (:object ?object-designator))
                                    ?robot-location-designator))))
    ;; pick-up-action
    (desig:desig-prop ;; spec:property
     ?action-designator (:pick-up-action ?some-pick-up-action-designator))
    (desig:current-designator ?some-pick-up-action-designator ?pick-up-action-designator)
)

  (<- (desig:action-grounding ?action-designator (deliver ?object-designator ?location-designator
                                                          ?robot-location-designator
                                                          ?place-action-designator))
    (spec:property ?action-designator (:type :delivering))
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; target
    (spec:property ?action-designator (:target ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location ?some-location-designator))
                   (desig:current-designator ?some-location-designator ?robot-location-designator))
              (desig:designator :location ((:reachable-for cram-pr2-description:pr2)
                                           (:location ?location-designator))
                                ?robot-location-designator)))
    ;; place-action
    (desig:desig-prop ;; spec:property
     ?action-designator (:place-action ?some-place-action-designator))
    (desig:current-designator ?some-place-action-designator ?place-action-designator))

  (<- (desig:action-grounding ?action-designator (transport
                                                  ?object-designator
                                                  ?fetching-location-designator
                                                  ?delivering-location-designator
                                                  ?arm
                                                  ?fetching-location-accessible))
    (spec:property ?action-designator (:type :transporting))
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    (spec:property ?action-designator (:location ?some-fetching-location-designator))
    (desig:current-designator ?some-fetching-location-designator ?fetching-location-designator)
    (spec:property ?action-designator (:target ?some-delivering-location-designator))
    (desig:current-designator ?some-delivering-location-designator ?delivering-location-designator)
    (or (spec:property ?action-designator (:arm ?arm))
        (equal ?arm NIL))
    (-> (desig:desig-prop ?fetching-location-designator (:in ?_))
        (equal ?fetching-location-accessible NIL)
        (equal ?fetching-location-accessible T))))
