(in-package :demo)

(def-fact-group whip-plans (desig:action-grounding)
;;###############################
;;###whip starting position
;;#############################

(<- (desig:action-grounding ?action-designator (whip ?resolved-action-designator))
      (spec:property ?action-designator (:type :mixing))
      ;; extract info from ?action-designator
      (spec:property ?action-designator (:object ?object-designator))
      (desig:current-designator ?object-designator ?object-target)
 ;     (spec:property ?current-object-desig (:type ?object-type))
 ;     (spec:property ?current-object-desig (:name ?object-name))
      
      (-> (spec:property ?action-designator (:arm ?arm))
          (and(true)
          (format "Step: ?arm defined - true.~%"
                ?direction))
          (man-int:robot-free-hand ?_ ?arm))
      ;wip check if arm is specified for object type
   (lisp-fun man-int:get-object-transform ?object-target ?object-transform)
    
    
; grasp spec not needed currently   (-> (spec:property ?action-designator (:grasp ?grasp))
;    
;        (and(true)
;        (format "grasp defined?~%"
;                ?direction))
;        (and (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)
;             (member ?grasp ?grasps)))

 ;later for bowl   
 ;   (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
 ;   (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

    ;; calculate trajectory
    (equal ?objects (?current-object-desig))
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory :mixing ?arm ?grasp T ?objects
                       ?left-mix-poses)
                       (lisp-fun man-int:get-traj-poses-by-label ?left-approach-pose :whip-approach
                       ?left-approach-poses)
;             (lisp-fun man-int:get-traj-poses-by-label ?left-mix- poses :whip-approach
 ;                      ?left-mix-poses))
            )
        (and (equal ?left-approach-poses NIL)
             (equal ?left-mix-poses NIL)))
    ;!!!#####curently only working in :right
    (-> (equal ?arm :right)
    (and (lisp-fun man-int:get-action-trajectory :mixing ?arm ?grasp T ?objects
                       ?right-mix-poses)
                           (lisp-fun man-int:get-traj-poses-by-label ?right-approach-pose :whip-approach
                       ?right-approach-poses)
;             (lisp-fun man-int:get-traj-poses-by-label ?right-mix- poses :whip-approach
 ;                      ?right-mix-poses))
            )
        (and (equal ?right-approach-poses NIL)
             (equal ?right-mix-poses NIL))
             )
             
;:or normal variable fine?
;   (-> (desi ?action-designator (:duration ?timer))
;       (default-whip-time)
;       (equal ?timer nil))
      
      ;;put together resulting action designator
      (desig:designator :action ((:type :mixing)
                                 (:object ?object-tool)
                                 (:object-type ?object-type)
                                 (:object-name  ?object-name)
                                (:object ?object-target)
                                 (:arm ?arm-tool)
                                (:effort ?effort)
                               (:grasp ?grasp)
                                ;(:left-mix-poses ?left-approach-poses)
                               ;(:right-mix-poses ?right-approach-poses)
                               ; (:duration ?timer))
                        ?resolved-action-designator))))
                        
;;######
;;HOLD CONTAINER -holding look at cut and pour designator.lisp
;;######


