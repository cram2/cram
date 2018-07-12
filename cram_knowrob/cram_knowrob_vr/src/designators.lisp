(in-package :kvr)

;; --- DESIG FACT GROUP --------------------------------------------------------------------
;; the fact groups were taken from Gayane Kazhoyan's "mobile-pick-and-place-plans" and were adapted to the needs of this thesis. (link to original: https://github.com/cram2/cram/blob/master/cram_common/cram_mobile_pick_place_plans/src/pick-place-designators.lisp)

(cram-prolog:def-fact-group ba-pnp-object-knowledge
    (object-rotationally-symmetric orientation-matters object-type-grasp)

  (cram-prolog:<- (orientation-matters ?object-type)
    (member ?object-type (:fork-blue-plastic)))
  (cram-prolog:<- (object-type-grasp :fork-blue-plastic :human-grasp))
  (cram-prolog:<- (object-type-grasp :weide-milch-small :human-grasp))
  (cram-prolog:<- (object-type-grasp :cup-eco-orange :human-grasp))
  (cram-prolog:<- (object-type-grasp :koelln-muesli-knusper-honig-nuss :human-grasp))
  (cram-prolog:<- (object-type-grasp :edeka-red-bowl :human-grasp)))

(cram-prolog:def-fact-group pick-and-place-plans (desig:action-grounding)
  (cram-prolog:<- (desig:action-grounding
                   ?action-designator
                   (pick-up ?current-object-desig ?arm
                            ?gripper-opening ?effort ?grasp
                            ?left-reach-poses ?right-reach-poses
                            ?left-lift-poses ?right-lift-poses))
    ;; extract info from ?action-designator
    (spec:property ?action-designator (:type :picking-up))
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    (spec:property ?current-object-desig (:name ?object-name))
    (cram-prolog:-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (and (cram-robot-interfaces:robot ?robot)
             (cram-robot-interfaces:arm ?robot ?arm)))
    ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
    (obj-int:object-type-grasp ?object-type ?grasp)
    (lisp-fun obj-int:get-object-type-gripping-effort ?object-type ?effort)
    (lisp-fun obj-int:get-object-type-gripper-opening ?object-type ?gripper-opening)
    (lisp-fun cram-object-interfaces:get-object-transform ?current-object-desig ?object-transform)
    (lisp-fun obj-int:get-object-grasping-poses
              ?object-name ?object-type :left ?grasp ?object-transform
              ?left-poses)
    (lisp-fun obj-int:get-object-grasping-poses
              ?object-name ?object-type :right ?grasp ?object-transform
              ?right-poses)
    (lisp-fun extract-pick-up-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses ?left-lift-poses ?right-lift-poses)))

  (cram-prolog:<- (desig:action-grounding ?action-designator (place ?current-object-designator ?arm
                                                        ?left-reach-poses ?right-reach-poses
                                                        ?left-put-poses ?right-put-poses
                                                        ?left-retract-poses ?right-retract-poses))
    (spec:property ?action-designator (:type :placing))
    (cram-prolog:-> (spec:property ?action-designator (:arm ?arm))
        (cram-prolog:-> (spec:property ?action-designator (:object ?object-designator))
            (or (cpoe:object-in-hand ?object-designator ?arm)
                (and (format "WARNING: Wanted to place an object ~a with arm ~a, ~
                              but it's not in the arm.~%" ?object-designator ?arm)
                     ;; (fail)
                     ))
            (cpoe:object-in-hand ?object-designator ?arm))
        (cram-prolog:-> (spec:property ?action-designator (:object ?object-designator))
            (cpoe:object-in-hand ?object-designator ?arm)
            (and (cram-robot-interfaces:robot ?robot)
                 (cram-robot-interfaces:arm ?robot ?arm)
                 (cpoe:object-in-hand ?object-designator ?arm))))
    (once (or (cpoe:object-in-hand ?object-designator ?arm)
              (spec:property ?action-designator (:object ?object-designator))))
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:type ?object-type))
    (spec:property ?current-object-designator (:name ?object-name))
    ;; infer missing information
    (obj-int:object-type-grasp ?object-type ?grasp)
    ;; take object-pose from action-designator target otherwise from object-designator pose
    (cram-prolog:-> (spec:property ?action-designator (:target ?location))
        (and (desig:current-designator ?location ?current-location-designator)
             (desig:designator-groundings ?current-location-designator ?poses)
             (member ?target-pose ?poses)
             (symbol-value cram-tf:*robot-base-frame* ?base-frame)
             (lisp-fun cram-tf:ensure-pose-in-frame ?target-pose ?base-frame :use-zero-time t
                       ?target-pose-in-base)
             (lisp-fun roslisp-utilities:rosify-underscores-lisp-name ?object-name ?tf-name)
             (lisp-fun cram-tf:pose-stamped->transform-stamped ?target-pose-in-base ?tf-name
                       ?target-transform))
        (lisp-fun cram-object-interfaces:get-object-transform
                  ?current-object-designator ?target-transform))
    (lisp-fun obj-int:get-object-grasping-poses
              ?object-name ?object-type :left ?grasp ?target-transform
              ?left-poses)
    (lisp-fun obj-int:get-object-grasping-poses
              ?object-name ?object-type :right ?grasp ?target-transform
              ?right-poses)
    (lisp-fun extract-place-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses ?left-put-poses ?right-put-poses
                                 ?left-retract-poses ?right-retract-poses))))

(cram-prolog:def-fact-group pick-and-place-atomic-actions (desig:action-grounding)

  (cram-prolog:<- (desig:action-grounding ?action-designator (move-arms-in-sequence ?left-poses ?right-poses))
    (or (spec:property ?action-designator (:type :reaching))
        (spec:property ?action-designator (:type :lifting))
        (spec:property ?action-designator (:type :putting))
        (spec:property ?action-designator (:type :retracting)))
    (once (or (spec:property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil)))
    (once (or (spec:property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil))))

  (cram-prolog:<- (desig:action-grounding ?action-designator (release ?left-or-right-or-both))
    (or (spec:property ?action-designator (:type :releasing))
        (spec:property ?action-designator (:type :opening)))
    (spec:property ?action-designator (:gripper ?left-or-right-or-both)))

  (cram-prolog:<- (desig:action-grounding ?action-designator (grip ?left-or-right-or-both ?object-grip-effort))
    (spec:property ?action-designator (:type :gripping))
    (spec:property ?action-designator (:gripper ?left-or-right-or-both))
    (spec:property ?action-designator (:effort ?object-grip-effort)))

  (cram-prolog:<- (desig:action-grounding ?action-designator (close-gripper ?left-or-right-or-both))
    (spec:property ?action-designator (:type :closing))
    (spec:property ?action-designator (:gripper ?left-or-right-or-both)))

  (cram-prolog:<- (desig:action-grounding ?action-designator (set-gripper-to-position
                                                  ?left-or-right-or-both ?position))
    (spec:property ?action-designator (:type :setting-gripper))
    (spec:property ?action-designator (:gripper ?left-or-right-or-both))
    (spec:property ?action-designator (:position ?position)))

  (cram-prolog:<- (desig:action-grounding ?action-designator (look-at :target ?location-designator))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:target ?location-designator)))
  (cram-prolog:<- (desig:action-grounding ?action-designator (look-at :frame ?frame))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:frame ?frame)))
  (cram-prolog:<- (desig:action-grounding ?action-designator (look-at :direction ?direction))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:direction ?direction)))
  (cram-prolog:<- (desig:action-grounding ?action-designator (look-at :object ?object-designator))
    (spec:property ?action-designator (:type :looking))
    (spec:property ?action-designator (:object ?object-designator)))

  (cram-prolog:<- (desig:action-grounding ?action-designator (navigate ?location-designator))
    (spec:property ?action-designator (:type :going))
    (spec:property ?action-designator (:target ?location-designator)))

  (cram-prolog:<- (desig:action-grounding ?action-designator (perceive ?object-designator))
    (spec:property ?action-designator (:type :detecting))
    (spec:property ?action-designator (:object ?object-designator))))
