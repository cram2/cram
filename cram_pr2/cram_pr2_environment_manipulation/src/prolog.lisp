(in-package :pr2-em)

(def-fact-group environment-manipulation (desig:action-grounding
                                          location-costmap:desig-costmap)
  
  (<- (desig:action-grounding ?action-designator (open-container ;; ?container-name
                                                                 ?arm
                                                                 ?gripper-opening
                                                                 ?left-reach-poses ?right-reach-poses
                                                                 ?left-lift-poses ?right-lift-poses))
    (spec:property ?action-designator (:type :opening))
    (spec:property ?action-designator (:object ?container-designator))
    (spec:property ?container-designator (:type :container))
    (spec:property ?container-designator (:name ?container-name))
    (spec:property ?container-designator (:part-of ?environment))
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (and (cram-robot-interfaces:robot ?robot)
             (cram-robot-interfaces:arm ?robot ?arm)))
    ;; infer missing information like ?gripper-opening, opening trajectory
    (lisp-fun obj-int:get-object-type-gripper-opening ?container-type ?gripper-opening)
    (lisp-fun obj-int:get-object-transform ?container-designator ?container-transform)
    (lisp-fun obj-int:get-object-grasping-poses ?container-name :container :left :front ?container-transform ?left-poses)
    (lisp-fun obj-int:get-object-grasping-poses ?container-name :container :right :front ?container-transform ?right-poses)
    (lisp-fun cram-mobile-pick-place-plans::extract-pick-up-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses ?left-lift-poses ?right-lift-poses))
    )

  (<- (desig:action-grounding ?action-designator (drive-to-and-open-container ?container-designator))
    (spec:property ?action-designator (:type :driving-and-opening))
    (spec:property ?action-designator (:object ?container-designator))
    (spec:property ?container-designator (:type :container))
    ;;(spec:property ?container-designator (:name ?container-name))
    ;;(spec:property ?container-designator (:part-of ?environment))
    )
  )
