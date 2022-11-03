(in-package :demo)

 
(def-fact-group pancake-actions (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (whisk ?resolved-action-designator))
    (spec:property ?action-designator (:type :whisking))
;;TODO: THIS STAYS THE SAME DONT CHANGE !!!! ++++++++++++++++++++++++++++++++
    ;; ==============   extract info from ?action-designator   ==============
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    (spec:property ?current-object-desig (:name ?object-name))
    (spec:property ?action-designator (:context ?context))
       
    
    ;; ==============   extract arms or set arms               ==============
    (man-int:arms-for-object-type ?object-type ?arms-for-object)
    (-> (equal ?arms-for-object nil)
        (-> (spec:property ?action-designator (:arm ?arm))
	    (true)
	    (format "Please set a specific arm ~%")))


    ;; ==============  infer missing information like ?grasp   ==============
    (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
    (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
    (-> (man-int:object-rotationally-symmetric ?object-type)
        (equal ?rotationally-symmetric t)
        (equal ?rotationally-symmetric nil))
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (and (member ?arm ?arms)
             (lisp-fun man-int:get-action-grasps ?object-type ?arm ?object-transform ?grasps)
             (member ?grasp ?grasps)))
    ;;(lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
    ;;(lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)

	(-> (equal ?reso nil)
	     ;TODO reso changes (and (lisp-fun get-default-reso ?reso)
	     	 (format "reso is:  ~a" ?reso);)
         (-> (spec:property ?action-designator (:reso ?reso))
         	(true)
	    ))
    
    ;;TODO: THIS STAYS THE SAME DONT CHANGE TILL HERE !!!! ++++++++++++

    
    ;; ==============  calculate trajectory with given grasp  ==============
    (equal ?objects (?current-object-desig))
    (-> (member :left ?arm)
	;;TODO change name here nad start with one pose only
	(and (lisp-fun man-int:get-action-trajectory :mixing
		       :left ?grasp T ?objects
		       ?left-trajectory)
	    ; (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :grip-container
		;       ?right-grip-container-poses)
	     (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :approach
		       ?left-approach-poses)
	     (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :start-mix
	      	       ?left-start-mix-poses)
	     (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :mid-mix
	      	       ?left-mid-mix-poses)
	     ;; (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :lift-pancake
	     ;; 	       ?left-lift-pancake-poses)
	     ;; (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :flip-pancake
	     ;; 	       ?left-flip-pancake-poses)
	     (lisp-fun get-arm-return :left ?left)
	     (format "arm: ~a ~% ?left-trajectory: ~a" ?left ?left-trajectory))
        ;;care u have to define the else here otherwise it cannot be resolved
	(and (equal ?left NIL)
	     (equal ?left-trajectory NIL)
	     (equal ?left-approach-poses NIL)
	     (equal ?left-start-mix-poses NIL)
	     (equal ?left-mid-mix-poses NIL)
	    ; (equal ?right-grip-container-poses NIL)
	     ;; (equal ?left-lift-pancake-poses NIL)
	     ;; (equal ?left-flip-pancake-poses NIL)
	     ))
    

    (-> (member :right ?arm)
	;;TODO change name here nad start with one pose only
	 (and (lisp-fun man-int:get-action-trajectory :mixing
			:right ?grasp T ?objects
			?right-trajectory)
	    ;  (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grip-container
		;	?left-grip-container-poses)
	      (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :approach
			?right-approach-poses)
	       (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :start-mix
	       		?right-start-mix-poses)
	      (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :mid-mix
	      		?right-mid-mix-poses)
	      ;; (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :lift-pancake
	      ;; 		?right-lift-pancake-poses)
	      ;; (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :flip-pancake
	      ;; 		?right-flip-pancake-poses)
	      (lisp-fun get-arm-return :right ?right)
     	     (format "arm: ~a ~% ?right-trajectory: ~a" ?right ?right-trajectory))
	;;care u have to define the else here otherwise it cannot be resolved
	 (and (equal ?right NIL)
	      (equal ?right-trajectory NIL)
	      (equal ?right-approach-poses NIL)
	      (equal ?right-start-mix-poses NIL)
	      (equal ?right-mid-mix-poses NIL)
	     ; (equal ?left-grip-container-poses NIL)
	      ;; (equal ?right-lif-pancake-poses NIL)
	      ;; (equal ?right-flip-pancake-poses NIL)
	      ))

    ;; ==============   put together resolving designator      ==============
    ;;TODO change name
    (desig:designator :action ((:type :whisking)
			       (:object ?object-designator)
			       (:object-type ?object-type)
			       (:object-name ?object-name)
			       (:arms ?arm)
		               (:grasp ?grasp)
			       (:context ?context)
			       (:reso ?reso)
			       ;;(:effort ?effort)
			       ;;(:gripper-opening ?gripper-opening)
			      ; (:left-grip-container-poses ?left-grip-container-poses)
			      ; (:right-grip-container-poses ?right-grip-container-poses)
		               (:left-approach-poses ?left-approach-poses)
			       (:right-approach-poses ?right-approach-poses)
			       (:left-start-mix-poses ?left-start-mix-poses)
			       (:right-start-mix-poses ?right-start-mix-poses)
                               (:left-mid-mix-poses ?left-mid-mix-poses)
                               (:right-mid-mix-poses ?right-mid-mix-poses)
			       ;; (:left-lift-pancake-poses ?left-lift-pancake-poses)
			       ;; (:right-lift-pancake-poses ?right-lift-pancake-poses)
			       ;; (:left-flip-pancake-poses ?left-flip-pancake-poses)
			       ;; (:right-flip-pancake-poses ?right-flip-pancake-poses)
			       )
                      ?resolved-action-designator)))
                      
(defun get-arm-return (?arm)
  ?arm)
