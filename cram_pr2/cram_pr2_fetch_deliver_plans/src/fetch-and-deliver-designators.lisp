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

  (<- (desig:action-grounding ?action-designator (go-without-collisions
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :navigating))
    (spec:property ?action-designator (:location ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    (desig:designator :action ((:type :navigating)
                               (:location ?location-designator))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (turn-towards
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :turning-towards))
    ;; target
    (spec:property ?action-designator (:target ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    ;; robot-location
    (lisp-fun calculate-robot-navigation-goal-towards-target ?location-designator
              ?robot-rotated-pose)
    (desig:designator :location ((:pose ?robot-rotated-pose)) ?robot-location)
    (desig:designator :action ((:type :turning-towards)
                               (:target ?location-designator)
                               (:robot-location ?robot-location))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (manipulate-environment
                                                  ?resolved-action-designator))
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
    (once (or (spec:property ?action-designator (:distance ?distance))
              (equal ?distance NIL)))
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location ?some-robot-location))
                   (desig:current-designator ?robot-location ?robot-location))
              (desig:designator :location ((:reachable-for cram-pr2-description:pr2)
                                           (:arm ?arm)
                                           (:object ?object-designator))
                                ?robot-location)))
    (desig:designator :action ((:type ?action-type)
                               (:object ?object-designator)
                               (:arm ?arm)
                               (:distance ?distance)
                               (:robot-location ?robot-location))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (search-for-object
                                                  ?resolved-action-designator))
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
                                ?location-to-stand)))
    (desig:designator :action ((:type :searching)
                               (:object ?object-designator)
                               (:location ?location-designator)
                               (:robot-location ?location-to-stand))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (fetch ?resolved-action-designator))
    (spec:property ?action-designator (:type :fetching))
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; arm
    (-> (spec:property ?action-designator (:arm ?arm))
        (equal ?arms (?arm))
        (-> (desig:desig-prop ?action-designator (:arms ?arms))
            (true)
            (equal ?arms NIL)))
    ;; grasp
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (equal ?grasps (?grasp))
        (-> (desig:desig-prop ?action-designator (:grasps ?grasps))
            (true)
            (equal ?grasps NIL)))
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
    (once (or (desig:desig-prop ;; spec:property
               ?action-designator (:pick-up-action ?some-pick-up-action-designator))
              (equal ?some-pick-up-action-designator NIL)))
    (desig:current-designator ?some-pick-up-action-designator ?pick-up-action-designator)
    (desig:designator :action ((:type :fetching)
                               (:object ?object-designator)
                               (:arms ?arms)
                               (:grasps ?grasps)
                               (:robot-location ?robot-location-designator)
                               (:pick-up-action ?pick-up-action-designator))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (deliver ?resolved-action-designator))
    (spec:property ?action-designator (:type :delivering))
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; arm
    (once (or (spec:property ?action-designator (:arm ?arm))
              (equal ?arm NIL)))
    ;; target
    (spec:property ?action-designator (:target ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location ?some-robot-loc-desig))
                   (desig:current-designator ?some-robot-loc-desig ?robot-location-designator))
              (desig:designator :location ((:reachable-for cram-pr2-description:pr2)
                                           (:location ?location-designator))
                                ?robot-location-designator)))
    ;; place-action
    (once (or (desig:desig-prop ;; spec:property
               ?action-designator (:place-action ?some-place-action-designator))
              (equal ?some-place-action-designator NIL)))
    (desig:current-designator ?some-place-action-designator ?place-action-designator)
    (desig:designator :action ((:type :delivering)
                               (:object ?object-designator)
                               (:arm ?arm)
                               (:target ?location-designator)
                               (:robot-location ?robot-location-designator)
                               (:place-action ?place-action-designator))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (transport ?resolved-action-designator))
    (spec:property ?action-designator (:type :transporting))
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; search location
    (spec:property ?action-designator (:location ?some-search-loc-desig))
    (desig:current-designator ?some-search-loc-desig ?search-location-designator)
    (-> (desig:desig-prop ?search-location-designator (:in ?_))
        (equal ?fetching-location-accessible NIL)
        (equal ?fetching-location-accessible T))
    ;; search location robot base
    (-> (desig:desig-prop ?action-designator
                          (:search-robot-location ?some-s-robot-loc-desig))
        (desig:current-designator ?some-s-robot-loc-desig
                                  ?search-robot-location-designator)
        (equal ?search-robot-location-designator NIL))
    ;; fetch location robot base
    (-> (desig:desig-prop ?action-designator
                          (:fetch-robot-location ?some-f-robot-loc-desig))
        (desig:current-designator ?some-f-robot-loc-desig
                                  ?fetch-robot-location-designator)
        (equal ?fetch-robot-location-designator NIL))
    ;; arm
    (-> (spec:property ?action-designator (:arm ?arm))
        (equal ?arms (?arm))
        (and (equal ?arm NIL)
             (-> (desig:desig-prop ?action-designator (:arms ?arms))
                 (true)
                 (equal ?arms NIL))))
    ;; grasp
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (equal ?grasps (?grasp))
        (and (equal ?grasp NIL)
             (-> (desig:desig-prop ?action-designator (:grasps ?grasps))
                 (true)
                 (equal ?grasps NIL))))
    ;; target location
    (spec:property ?action-designator
                   (:target ?some-delivering-location-designator))
    (desig:current-designator ?some-delivering-location-designator
                              ?delivering-location-designator)
    ;; deliver location robot base
    (-> (desig:desig-prop ?action-designator
                          (:deliver-robot-location ?some-d-robot-loc-desig))
        (desig:current-designator ?some-d-robot-loc-desig
                                  ?deliver-robot-location-designator)
        (equal ?deliver-robot-location-designator NIL))
    ;; resulting action desig
    (desig:designator :action ((:type :transporting)
                               (:object ?object-designator)
                               (:search-location ?search-location-designator)
                               (:search-robot-location ?search-robot-location-designator)
                               (:fetch-robot-location ?fetch-robot-location-designator)
                               (:arm ?arm)
                               (:grasp ?grasp)
                               (:arms ?arms)
                               (:grasps ?grasps)
                               (:deliver-location ?delivering-location-designator)
                               (:deliver-robot-location ?deliver-robot-location-designator)
                               (:search-location-accessible ?fetching-location-accessible))
                      ?resolved-action-designator)))
