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

(in-package :fd-plans)

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
    ;; Check if robots holds one object with both arms
    ;; get robot arms and how many arms the robot has
    (setof ?arm (rob-int:arm ?_ ?arm) ?rob-arms)
    (length ?rob-arms ?number-of-arms)
    ;; check if the robot with more than one arm holds only one object
    (-> (and (cpoe:object-in-hand ?object-designator ?arm ?grasp)
             (> ?number-of-arms 1)
             (forall (member ?rob-arm ?rob-arms)
                     (cpoe:object-in-hand ?object-designator ?rob-arm)))
        (equal ?park-arms NIL)
        (equal ?park-arms T))
    (desig:designator :action ((:type :navigating)
                               (:location ?location-designator)
                               (:park-arms ?park-arms))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (turn-towards
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :turning-towards))
    ;; target
    (spec:property ?action-designator (:target ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    ;; robot-location
    (-> (man-int:location-always-reachable ?location-designator)
        (and (lisp-fun cram-tf:robot-current-pose ?robot-rotated-pose)
             (equal ?target-always-reachable T))
        (and (lisp-fun calculate-robot-navigation-goal-towards-target ?location-designator
                       ?robot-rotated-pose)
             (equal ?target-always-reachable NIL)))
    (desig:designator :location ((:pose ?robot-rotated-pose)) ?robot-location)
    (desig:designator :action ((:type :turning-towards)
                               (:target ?location-designator)
                               (:robot-location ?robot-location)
                               (:target-always-reachable ?target-always-reachable))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (manipulate-environment
                                                  ?resolved-action-designator))
    (or (spec:property ?action-designator (:type :accessing))
        (spec:property ?action-designator (:type :sealing)))
    (spec:property ?action-designator (:type ?action-type))
    (rob-int:robot ?robot)
    ;; location
    (spec:property ?action-designator (:location ?some-location-designator))
    (desig:current-designator ?some-location-designator ?location-designator)
    ;; object
    (once (or (and (or (spec:property ?location-designator (:in ?some-object-designator))
                       (spec:property ?location-designator (:above ?some-object-designator)))
                   (equal ?object-accessible nil))
              (and (spec:property ?location-designator (:on ?some-object-designator))
                   (equal ?object-accessible t))))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; location of the object that we are trying to access
    (once (or (spec:property ?object-designator (:location ?object-location-desig))
              (equal ?object-location-desig nil)))
    ;; arms
    (-> (spec:property ?action-designator (:arms ?arms))
        (true)
        (setof ?arm (man-int:robot-free-hand ?robot ?arm) ?arms))
    ;; distance
    (once (or (spec:property ?action-designator (:distance ?distance))
              (equal ?distance NIL)))
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location
                                                      ?some-robot-location))
                   (desig:current-designator ?robot-location ?robot-location))
              (desig:designator :location ((:reachable-for ?robot)
                                           ;; (:arm ?arm)
                                           (:object ?object-designator))
                                ?robot-location)))
    (desig:designator :action ((:type ?action-type)
                               (:object ?object-designator)
                               (:object-accessible ?object-accessible)
                               (:object-location ?object-location-desig)
                               (:arms ?arms)
                               (:distance ?distance)
                               (:robot-location ?robot-location))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (search-for-object
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :searching))
    (rob-int:robot ?robot)
    ;; searching for an object or for a location
    (once (or (spec:property ?action-designator (:object ?some-obj-desig))
              (and (spec:property ?action-designator (:location ?some-loc-desig))
                   (man-int:location-reference-object ?some-loc-desig
                                                      ?some-obj-desig))))
    (desig:current-designator ?some-obj-desig ?object-designator)
    ;; context
    (once (or (spec:property ?action-designator (:context ?context))
              (equal ?context NIL)))
    ;; where to look for the given object or the given location's reference object
    (-> (and (spec:property ?object-designator (:location ?some-location-designator))
             (not (equal ?some-location-designator NIL)))
        (and (desig:current-designator ?some-location-designator ?location-designator)
             (equal ?object-designator ?object-designator-with-location))
        (and (spec:property ?object-designator (:type ?object-type))
             (rob-int:environment-name ?environment)
             (lisp-fun man-int:get-object-likely-location
                       ?object-type ?environment nil ?context ?location-designator)
             (equal ?new-props ((:location ?location-designator)))
             (lisp-fun desig:extend-designator-properties
                       ?object-designator ?new-props
                       ?object-designator-with-location)
             (lisp-pred desig:equate
                        ?object-designator ?object-designator-with-location)))
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location
                                                      ?some-location-to-stand))
                   (desig:current-designator ?some-location-to-stand
                                             ?location-to-stand))
              (desig:designator :location ((:visible-for ?robot)
                                           (:location ?location-designator)
                                           (:object ?object-designator))
                                ?location-to-stand)))
    (desig:designator :action ((:type :searching)
                               (:object ?object-designator-with-location)
                               (:location ?location-designator)
                               (:robot-location ?location-to-stand))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (fetch ?resolved-action-designator))
    (spec:property ?action-designator (:type :fetching))
    (rob-int:robot ?robot)
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; arms
    (-> (spec:property ?action-designator (:arms ?arms))
        (true)
        (setof ?arm (man-int:robot-free-hand ?robot ?arm) ?arms))
    ;; grasps
    (-> (spec:property ?action-designator (:grasps ?grasps))
        (true)
        ;; we do not ask for grasps because they are arm-specific!
        ;; therefore, projection reasoning is essential for these plans
        ;; (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform
        ;;           ?grasps)
        (equal ?grasps NIL))
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location
                                                      ?some-location-designator))
                   (desig:current-designator ?some-location-designator
                                             ?robot-location-designator))
              (desig:designator :location ((:reachable-for ?robot)
                                           ;; ?arm is not available because we're sampling
                                           ;; (:arm ?arm)
                                           (:object ?object-designator)
                                           ;; have to add the visibility
                                           ;; constraint as he reperceives
                                           ;; each time before grasping
                                           (:visible-for ?robot))
                                ?robot-location-designator)))
    ;; if the object is in the hand or its reference object is in the hand
    ;; we need to bring the hand closer to the other hand, e.g., bring to front
    (-> (man-int:object-or-its-reference-in-hand ?object-designator ?object-hand)
        (equal ?object-in-hand T)
        (and (equal ?object-in-hand NIL)
             (equal ?object-hand NIL)))
    ;; look-location
    (once (or (and (spec:property ?action-designator (:look-location
                                                      ?some-look-loc-desig))
                   (desig:current-designator ?some-look-loc-desig
                                             ?look-location-designator))
              (-> (or (equal ?object-in-hand NIL) (equal ?object-hand NIL))
                  (desig:designator :location ((:of ?object-designator))
                                    ?look-location-designator)
                  (and (desig:designator :object ((:part-of ?robot)
                                                  (:link :robot-tool-frame)
                                                  (:which-link ?object-hand))
                                         ?object-hand-designator)
                       (desig:designator :location ((:of ?object-hand-designator))
                                         ?look-location-designator)))))
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
                               (:look-location ?look-location-designator)
                               (:pick-up-action ?pick-up-action-designator)
                               (:object-in-hand ?object-in-hand)
                               (:object-hand ?object-hand))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (deliver ?resolved-action-designator))
    (spec:property ?action-designator (:type :delivering))
    (rob-int:robot ?robot)
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; arm
    (once (or (spec:property ?action-designator (:arm ?arm))
              (equal ?arm NIL)))
    ;; context
    (once (or (spec:property ?action-designator (:context ?context))
              (equal ?context NIL)))
    ;; target
    (-> (and (spec:property ?action-designator (:target ?some-location-designator))
             (not (equal ?some-location-designator NIL)))
        (desig:current-designator ?some-location-designator ?location-designator)
        (and (spec:property ?object-designator (:type ?object-type))
             (rob-int:environment-name ?environment)
             (lisp-fun man-int:get-object-destination
                       ?object-type ?environment nil ?context ?location-designator)))
    ;; target stable? or have to check stability first?
    (-> (man-int:location-always-stable ?location-designator)
        (equal ?target-stable T)
        (equal ?target-stable NIL))
    ;; also, the target location can be w.r.t. other object, which can be in hand,
    ;; in which case we need to bring the other object hand closer
    (-> (and (man-int:location-reference-object ?location-designator ?target-obj)
             (cpoe:object-in-hand ?target-obj ?target-hand))
        (equal ?target-in-hand T)
        (and (equal ?target-in-hand NIL)
             (equal ?target-hand NIL)))
    ;; robot-location
    (once (or (and (spec:property ?action-designator (:robot-location
                                                      ?some-robot-loc-desig))
                   (desig:current-designator ?some-robot-loc-desig
                                             ?robot-location-designator))
              (desig:designator :location ((:reachable-for ?robot)
                                           (:object ?object-designator)
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
                               (:place-action ?place-action-designator)
                               (:target-stable ?target-stable)
                               (:target-in-hand ?target-in-hand)
                               (:target-hand ?target-hand))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (transport
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :transporting))
    ;; object
    (spec:property ?action-designator (:object ?some-object-designator))
    (desig:current-designator ?some-object-designator ?object-designator)
    ;; context
    (once (or (spec:property ?action-designator (:context ?context))
              (equal ?context NIL)))
    ;; search location
    (-> (and (spec:property ?object-designator (:location ?some-search-loc-desig))
             (not (equal ?some-search-loc-desig NIL)))
        (and (desig:current-designator ?some-search-loc-desig
                                       ?search-location-designator)
             (equal ?object-designator-with-location ?object-designator))
        (and (spec:property ?object-designator (:type ?object-type))
             (rob-int:environment-name ?environment)
             (lisp-fun man-int:get-object-likely-location
                       ?object-type ?environment nil ?context
                       ?search-location-designator)
             (equal ?new-props ((:location ?search-location-designator)))
             (lisp-fun desig:extend-designator-properties
                       ?object-designator ?new-props
                       ?object-designator-with-location)
             (lisp-pred desig:equate
                        ?object-designator ?object-designator-with-location)))
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
    ;; arms
    (-> (spec:property ?action-designator (:arms ?arms))
        (true)
        (equal ?arms NIL))
    ;; grasps
    (-> (spec:property ?action-designator (:grasps ?grasps))
        (true)
        (equal ?grasps NIL))
    ;; deliver location
    (-> (and (spec:property ?action-designator
                            (:target ?some-delivering-location-designator))
             (not (equal ?some-delivering-location-designator NIL)))
        (desig:current-designator ?some-delivering-location-designator
                                  ?delivering-location-designator)
        (and (spec:property ?object-designator (:type ?object-type))
             (rob-int:environment-name ?environment)
             (lisp-fun man-int:get-object-destination
                       ?object-type ?environment nil ?context
                       ?delivering-location-designator)))
    ;; deliver location robot base
    (-> (desig:desig-prop ?action-designator
                          (:deliver-robot-location ?some-d-robot-loc-desig))
        (desig:current-designator ?some-d-robot-loc-desig
                                  ?deliver-robot-location-designator)
        (equal ?deliver-robot-location-designator NIL))
    ;; resulting action desig
    (desig:designator
     :action
     ((:type :transporting)
      (:object ?object-designator-with-location)
      (:context ?context)
      (:search-location ?search-location-designator)
      (:search-robot-location ?search-robot-location-designator)
      (:fetch-robot-location ?fetch-robot-location-designator)
      (:arms ?arms)
      (:grasps ?grasps)
      (:deliver-location ?delivering-location-designator)
      (:deliver-robot-location ?deliver-robot-location-designator))
     ?resolved-action-designator)))
