(in-package :demo)

(def-fact-group whip-designator (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (whip ?resolved-action-designator))
    (spec:property ?action-designator (:type :mixing))
    ;; extract info from ?action-designator
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    (spec:property ?current-object-desig (:name ?object-name))
    
      (-> (spec:property ?action-designator (:arms ?arms))
	  (and (true)
	       (format "arm true"))
        (and (man-int:robot-free-hand ?_ ?arm)
             (equal ?arms (?arm))))
    
     ;wip check if arm is specified for object type
     (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
    
    
				
  ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
     (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
     (-> (man-int:object-rotationally-symmetric ?object-type)
	 (equal ?rotationally-symmetric t)
	 (equal ?rotationally-symmetric nil))
     (-> (spec:property ?action-designator (:grasp ?grasp))
	 (true)
	 (and (member ?arm ?arms)
	      (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)
	      (member ?grasp ?grasps)))

					;later for bowl   
    (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
    (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

    ;; calculate trajectory
    (equal ?objects (?current-object-desig))
    (-> (member :left ?arms)
	;;get all the poses as list
        (and (lisp-fun man-int:get-action-trajectory :mixing :left ?grasp T ?objects
                       ?left-mix-poses)
	     ;;get from the list the pose with key word :whip-approach
	     (lisp-fun man-int:get-traj-poses-by-label ?left-mix-poses :whip-approach
                       ?left-whip-approach-poses)
					;             (lisp-fun man-int:get-traj-poses-by-label ?left-mix- poses :whip-approach
					;                      ?left-mix-poses))
	     )
        (and (equal ?left-whip-approach-poses NIL)
             (equal ?left-mix-poses NIL)))
					;!!!#####curently only working in :right
    (-> (member :right ?arms)
	(and (lisp-fun man-int:get-action-trajectory :mixing :right ?grasp T ?objects
                       ?right-mix-poses)
	     (lisp-fun man-int:get-traj-poses-by-label ?right-mix-pose :whip-approach
                       ?right-whip-approach-poses)
					;             (lisp-fun man-int:get-traj-poses-by-label ?right-mix- poses :whip-approach
					;                      ?right-mix-poses))
	     )
        (and (equal ?right-whip-approach-poses NIL)
             (equal ?right-mix-poses NIL))
	)
     (-> (desig:desig-prop ?action-designator (:collision-mode ?collision-mode))
        (true)
        (equal ?collision-mode nil))
    
					;:or normal variable fine?
					;   (-> (desi ?action-designator (:duration ?timer))
					;       (default-whip-time)
					;       (equal ?timer nil))
    
    ;;put together resulting action designator
    (desig:designator :action ((:type :mixing)
			       (:object ?current-object-desig)
			       (:object-type ?object-type)
			       (:object-name  ?object-name)
			       (:arms ?arms)
			       (:effort ?effort)
                               (:grasp ?grasp)
			       (:left-whip-approach-poses ?left-whip-approach-poses)
			       (:right-whip-approach-poses ?right-whip-approach-poses)
					; (:duration ?timer))
			       (:collision-mode ?collision-mode))
		      ?resolved-action-designator)))
                        
;;######
;;HOLD CONTAINER -holding look at cut and pour designator.lisp
;;######


