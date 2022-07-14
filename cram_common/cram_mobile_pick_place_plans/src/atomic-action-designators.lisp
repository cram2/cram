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

  (<- (desig:action-grounding ?action-designator (go-to-target ?resolved-action-designator))
    (spec:property ?action-designator (:type :going))
    (spec:property ?action-designator (:target ?some-location-designator))
    (once (or (spec:property ?action-designator (:speed ?speed))
              (equal ?speed nil)))
    (desig:current-designator ?some-location-designator ?location-designator)
    (desig:designator-groundings ?location-designator ?poses)
    (member ?pose-stamped ?poses)
    (desig:designator :action ((:type :going)
                               (:pose ?pose-stamped)
                               (:speed ?speed)
                               (:slow-speed :slow))
                      ?resolved-action-designator))



  (<- (desig:action-grounding ?action-designator (go-with-torso ?action-designator))
    (spec:property ?action-designator (:type :moving-torso))
    (spec:property ?action-designator (:joint-angle ?_)))



  (<- (infer-prefer-base ?action-designator ?prefer-base)
    ;; infer if the robot should prefer to move the base more than arms
    ;; prefer-base is only true when we're opening prismatic containers
    (-> (and (spec:property ?action-designator (:type :pulling))
             (spec:property ?action-designator (:container-object ?container-designator))
             (spec:property ?container-designator (:type ?container-type))
             (man-int:object-type-subtype :container-prismatic ?container-type))
        (equal ?prefer-base t)
        (equal ?prefer-base nil)))

  (<- (infer-move-base ?action-designator ?move-base)
    ;; infer if we should move the base at all or not
    ;; we shouldn't move the base if we're getting something of our own back
    ;; or if we're putting something on our own back
    (-> (and (or (spec:property ?action-designator (:type :reaching))
                 (spec:property ?action-designator (:type :grasping))
                 (spec:property ?action-designator (:type :putting)))
             (or (spec:property ?action-designator (:location ?location-designator))
                 (and (spec:property ?action-designator
                                     (:object ?some-object-designator))
                      (desig:current-designator ?some-object-designator
                                                ?object-designator)
                      (spec:property ?object-designator
                                     (:location ?location-designator))))
             (man-int:location-always-reachable ?location-designator))
        (equal ?move-base nil)
        (equal ?move-base t)))

  (<- (infer-align-planes ?action-designator ?align-planes-left ?align-planes-right)
    ;; infer if we should keep the object in hand upright
    ;; that should currently always happen if we have an object in hand
    (-> (cpoe:object-in-hand ?_ :left)
        (equal ?align-planes-left t)
        (equal ?align-planes-left nil))
    (-> (cpoe:object-in-hand ?_ :right)
        (equal ?align-planes-right t)
        (equal ?align-planes-right nil)))

  (<- (infer-motion-flags ?action-designator
                          ?prefer-base ?move-base
                          ?align-planes-left ?align-planes-right)
    (infer-prefer-base ?action-designator ?prefer-base)
    (infer-move-base ?action-designator ?move-base)
    (infer-align-planes ?action-designator ?align-planes-left ?align-planes-right))

  (<- (desig:action-grounding ?action-designator (move-arms-in-sequence
                                                  ?resolved-action-designator))
    (or (spec:property ?action-designator (:type :reaching))
        (spec:property ?action-designator (:type :retracting))
        (spec:property ?action-designator (:type :lifting))
        (spec:property ?action-designator (:type :approaching))
        (spec:property ?action-designator (:type :tilting))
        (spec:property ?action-designator (:type :retracting))
        (spec:property ?action-designator (:type :cutting)))
    (spec:property ?action-designator (:type ?action-type))
    (once (or (spec:property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (spec:property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil)))
    (once (or (spec:property ?action-designator (:collision-mode ?collision))
              (equal ?collision :allow-all)))
    (infer-motion-flags ?action-designator
                        ?_ ?move-base ?align-planes-left ?align-planes-right)
    (desig:designator :action ((:type ?action-type)
                               (:left-poses ?left-poses)
                               (:right-poses ?right-poses)
                               (:collision-mode ?collision)
                               (:move-base ?move-base)
                               (:align-planes-left ?align-planes-left)
                               (:align-planes-right ?align-planes-right))
                      ?resolved-action-designator))

  (<- (desig:action-grounding ?action-designator (move-arms-in-sequence
                                                  ?resolved-action-designator))
    (or (spec:property ?action-designator (:type :retracting)))
    (spec:property ?action-designator (:type ?action-type))
    (once (or (spec:property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (spec:property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil)))
    (once (or (spec:property ?action-designator (:collision-mode ?collision))
              (equal ?collision :allow-hand)))
    (infer-motion-flags ?action-designator
                        ?_ ?move-base ?align-planes-left ?align-planes-right)
    (desig:designator :action ((:type ?action-type)
                               (:left-poses ?left-poses)
                               (:right-poses ?right-poses)
                               (:collision-mode ?collision)
                               (:move-base ?move-base)
                               (:align-planes-left ?align-planes-left)
                               (:align-planes-right ?align-planes-right))
                      ?resolved-action-designator))

  (<- (desig:action-grounding ?action-designator (move-arms-in-sequence
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :grasping))
    (spec:property ?action-designator (:type ?action-type))
    (spec:property ?action-designator (:object ?object-designator))
    (spec:property ?object-designator (:name ?object-name))
    (once (or (spec:property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (spec:property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil)))
    (infer-motion-flags ?action-designator
                        ?prefer-base ?move-base
                        ?align-planes-left ?align-planes-right)
    ;; infer collision-object-b and collision-object-b-link
    (-> (cpoe:object-in-hand ?object-designator ?_ ?_ ?_)
        (and (rob-int:robot ?robot)
             (equal ?collision-object-b ?robot)
             (equal ?object-link ?object-name))
        (and (equal ?collision-object-b ?object-name)
             (once (or (spec:property ?action-designator (:link ?object-link))
                       (equal ?object-link nil)))))
    (desig:designator :action ((:type ?action-type)
                               (:left-poses ?left-poses)
                               (:right-poses ?right-poses)
                               (:collision-mode :allow-hand)
                               (:collision-object-b ?collision-object-b)
                               (:collision-object-b-link ?object-link)
                               (:prefer-base ?prefer-base)
                               (:move-base ?move-base)
                               (:align-planes-left ?align-planes-left)
                               (:align-planes-right ?align-planes-right))
                      ?resolved-action-designator))

  (<- (desig:action-grounding ?action-designator (move-arms-in-sequence
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :putting))
    (spec:property ?action-designator (:object ?object-designator))
    (spec:property ?object-designator (:name ?object-name))
    (-> (spec:property ?action-designator (:supporting-object ?other-object-designator))
        (and (or (spec:property ?other-object-designator (:name ?other-object-name))
                 (spec:property ?other-object-designator (:part-of ?other-object-name))
                 (equal ?other-object-name nil))
             (or (spec:property ?other-object-designator (:urdf-name ?object-link))
                 (equal ?object-link nil)))
        (and (equal ?other-object-name nil)
             (equal ?object-link nil)))
    (once (or (spec:property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (spec:property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil)))
    (infer-motion-flags ?action-designator
                        ?_ ?move-base ?align-planes-left ?align-planes-right)
    ;; putting should actually allow hand and attached if grasping allows hand
    (desig:designator :action ((:type :putting)
                               (:left-poses ?left-poses)
                               (:right-poses ?right-poses)
                               (:collision-mode :allow-all;; :allow-attached
                                                )
                               (:collision-object-b ?other-object-name)
                               (:collision-object-b-link ?object-link)
                               (:collision-object-a ?object-name)
                               (:move-base ?move-base)
                               (:align-planes-left ?align-planes-left)
                               (:align-planes-right ?align-planes-right))
                      ?resolved-action-designator))

  (<- (desig:action-grounding ?action-designator (manipulate-environment
                                                  ?resolved-action-designator))
    (or (and (spec:property ?action-designator (:type :pulling))
             (equal ?collision-mode :allow-hand))
        (and (spec:property ?action-designator (:type :pushing))
             (equal ?collision-mode :allow-arm)))
    (or (and (spec:property ?action-designator (:left-poses ?poses))
             (equal ?arm :left))
        (and (spec:property ?action-designator (:right-poses ?poses))
             (equal ?arm :right)))
    (spec:property ?action-designator (:type ?action-type))
    (spec:property ?action-designator (:object ?environment-object-designator))
    (spec:property ?environment-object-designator (:name ?environment-name))
    (spec:property ?action-designator (:link ?handle-link))
    (once (or (spec:property ?action-designator (:distance ?joint-angle))
              (equal ?joint-angle NIL)))
    ;; infer the missing parameters
    (infer-motion-flags ?action-designator
                        ?prefer-base ?move-base
                        ?align-planes-left ?align-planes-right)
    (desig:designator :action ((:type ?action-type)
                               (:arm ?arm)
                               (:poses ?poses)
                               (:distance ?joint-angle)
                               (:collision-mode ?collision-mode)
                               (:collision-object-b ?environment-name)
                               (:collision-object-b-link ?handle-link)
                               (:prefer-base ?prefer-base)
                               (:move-base ?move-base)
                               (:align-planes-left ?align-planes-left)
                               (:align-planes-right ?align-planes-right))
                      ?resolved-action-designator))

  (<- (desig:action-grounding ?action-designator (move-arms-into-configuration
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :positioning-arm))
    (rob-int:robot ?robot)
    (-> (spec:property ?action-designator (:left-configuration ?left-config))
        (man-int:joint-state-for-arm-config ?robot ?left-config :left
                                            ?left-joint-states)
        (equal ?left-joint-states nil))
    (-> (spec:property ?action-designator (:right-configuration ?right-config))
        (man-int:joint-state-for-arm-config ?robot ?right-config :right
                                            ?right-joint-states)
        (equal ?right-joint-states nil))
    (infer-align-planes ?action-designator ?align-planes-left ?align-planes-right)
    (desig:designator :action ((:type :positioning-arm)
                               (:left-joint-states ?left-joint-states)
                               (:right-joint-states ?right-joint-states)
                               (:align-planes-left ?align-planes-left)
                               (:align-planes-right ?align-planes-right))
                      ?resolved-action-designator))


  (<- (desig:action-grounding ?action-designator (release ?resolved-action-designator))
    (spec:property ?action-designator (:type :releasing))
    (once (or (spec:property ?action-designator (:gripper ?gripper))
              (setof ?arm (rob-int:arm ?robot ?arm) ?gripper)))
    (once (or (spec:property ?action-designator (:object ?object))
              (equal ?object NIL)))
    (desig:designator :action ((:type :releasing)
                               (:gripper ?gripper)
                               (:object ?object))
                      ?resolved-action-designator))

  (<- (desig:action-grounding ?action-designator (grip ?action-designator))
    (spec:property ?action-designator (:type :gripping))
    (spec:property ?action-designator (:gripper ?_))
    (not (spec:property ?action-designator (:object ?_)))
    (once (or (spec:property ?action-designator (:effort ?_))
              (true))))

  (<- (desig:action-grounding ?action-designator (grip ?action-designator))
    (spec:property ?action-designator (:type :gripping))
    (spec:property ?action-designator (:gripper ?_))
    (spec:property ?action-designator (:object ?object-designator))
    ;; TODO: if grasp is not given, calculate it from relative offset
    ;; something like
    ;; (lisp-fun man-int:calculate-grasp ?object-desig-name ?gripper)
    (once (or (spec:property ?action-designator (:grasp ?_))
              (true)))
    (once (or (spec:property ?action-designator (:effort ?_))
              (true))))

  (<- (desig:action-grounding ?action-designator (set-gripper-to-position ?action-designator))
    (spec:property ?action-designator (:type :setting-gripper))
    (spec:property ?action-designator (:gripper ?_))
    (spec:property ?action-designator (:position ?_)))

  (<- (desig:action-grounding ?action-designator (open-or-close-gripper ?action-designator))
    (or (spec:property ?action-designator (:type :closing-gripper))
        (spec:property ?action-designator (:type :opening-gripper)))
    (spec:property ?action-designator (:gripper ?_)))



  (<- (desig:action-grounding ?action-designator (look-at ?resolved-action-designator))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:target ?location-designator))
    (desig:designator-groundings ?location-designator ?poses)
    (member ?pose-stamped ?poses)
    ;; (-> (spec:property ?action-designator (:camera ?camera))
    ;;     (equal ?camera :head)
    ;;     (true))
    (desig:designator :action ((:type :looking)
                               (:pose ?pose-stamped)
                               (:camera :head))
                      ?resolved-action-designator))

  (<- (desig:action-grounding ?action-designator (look-at ?resolved-action-designator))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-designator)
    (lisp-fun man-int:get-object-pose ?current-object-designator ?pose-stamped)
    ;; (-> (spec:property ?action-designator (:camera ?camera))
    ;;     (equal ?camera :head)
    ;;     (true))
    (desig:designator :action ((:type :looking)
                               (:pose ?pose-stamped)
                               (:camera :head))
                      ?resolved-action-designator))

  (<- (desig:action-grounding ?action-designator (look-at ?resolved-action-designator))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:frame ?frame))
    (lisp-fun cl-transforms:make-identity-pose ?identity-pose)
    (lisp-fun cl-transforms-stamped:pose->pose-stamped ?frame 0.0 ?identity-pose
              ?pose-stamped)
    ;; (-> (spec:property ?action-designator (:camera ?camera))
    ;;     (equal ?camera :head)
    ;;     (true))
    (desig:designator :action ((:type :looking)
                               (:pose ?pose-stamped)
                               (:camera :head))
                      ?resolved-action-designator))

  (<- (desig:action-grounding ?action-designator (look-at ?resolved-action-designator))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:direction ?direction))
    (rob-int:robot ?robot)
    (-> (rob-int:robot-pose ?robot :neck ?_ ?direction ?pose-stamped)
        (true)
        (equal ?pose-stamped nil))
    (-> (rob-int:robot-joint-states ?robot :neck ?_ ?direction ?joint-states)
        (true)
        (equal ?joint-states nil))
    (-> (and (equal ?pose-stamped nil) (equal ?joint-states nil))
        (format "WARNING: in a LOOKING action DIRECTION ~a was unknown.~%"
                ?direction)
        (true))
    ;; (-> (spec:property ?action-designator (:camera ?camera))
    ;;     (equal ?camera :head)
    ;;     (true))
    (desig:designator :action ((:type :looking)
                               (:pose ?pose-stamped)
                               (:joint-states ?joint-states)
                               (:camera :head))
                      ?resolved-action-designator))



  (<- (desig:action-grounding ?action-designator (detect ?action-designator))
    (spec:property ?action-designator (:type :detecting))
    (spec:property ?action-designator (:object ?_)))



  (<- (desig:action-grounding ?action-designator (monitor-joint-state
                                                  ?resolved-action-designator))
    (spec:property ?action-designator (:type :monitoring-joint-state))
    (spec:property ?action-designator (:gripper ?left-or-right))
    (rob-int:robot ?robot)
    (rob-int:gripper-joint ?robot ?left-or-right ?joint-name)
    (rob-int:gripper-minimal-position ?robot ?left-or-right ?minimum)
    (rob-int:gripper-convergence-delta ?robot ?left-or-right ?delta)
    (lisp-fun + ?minimum ?delta ?joint-angle-threshold)
    (lisp-fun symbol-function < ?function)
    (desig:designator :action ((:type :monitoring-joint-state)
                               (:joint-name ?joint-name)
                               (:joint-angle-threshold ?joint-angle-threshold)
                               (:function ?function))
                      ?resolved-action-designator))



  (<- (desig:action-grounding ?action-designator (wait ?action-designator))
    (spec:property ?action-designator (:type :waiting))
    (spec:property ?action-designator (:duration ?_))))
