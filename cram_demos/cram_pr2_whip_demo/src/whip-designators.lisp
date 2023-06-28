(in-package :demo)

 
(def-fact-group whip-actions (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (whisk ?resolved-action-designator))
    (spec:property ?action-designator (:type :whisking))
;;TODO: THIS STAYS THE SAME DONT CHANGE !!!! ++++++++++++++++++++++++++++++++
    ;; ==============   extract info from ?action-designator   ==============
    (spec:property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-desig)
    (spec:property ?current-object-desig (:type ?object-type))
    (spec:property ?current-object-desig (:name ?object-name))
    (spec:property ?action-designator (:context ?context))

    (-> (spec:property ?action-designator (:reso ?reso))
        (true)
        (?reso format "no reso specified"))

    (spec:property ?action-designator (:source ?source-designator))
    (desig:current-designator ?source-designator ?current-source-desig)
    (spec:property ?current-source-desig (:type ?source-type))
    
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
    
    ;;TODO: THIS STAYS THE SAME DONT CHANGE TILL HERE !!!! ++++++++++++

    
    ;; ==============  calculate trajectory with given grasp  ==============
    (equal ?object (?current-object-desig))
    (-> (member :left ?arm)
	;;TODO change name here nad start with one pose only
	(and
         (lisp-fun man-int:get-action-trajectory :mixing
		       :left ?grasp T ?object :context ?context :reso ?reso :tool-object-type ?source-type :container-arm :right
		       ?left-trajectory)
	    ; (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :grip-container
		;       ?right-grip-container-poses)
	     (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :approach
		       ?left-approach-poses)
	     (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :start-mix
	      	       ?left-start-mix-poses)
	     (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :mid-mix
	      	       ?left-mid-mix-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :end-mix
                       ?left-end-mix-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retract
                       ?left-retract-poses)

	     (lisp-fun get-arm-return :left ?left)
	     (format "arm: ~a ~% ?left-trajectory: ~a" ?left ?left-trajectory))
        ;;care u have to define the else here otherwise it cannot be resolved
	(and (equal ?left NIL)
	     (equal ?left-trajectory NIL)
	     (equal ?left-approach-poses NIL)
	     (equal ?left-start-mix-poses NIL)
	     (equal ?left-mid-mix-poses NIL)
             (equal ?left-end-mix-poses NIL)
	     (equal ?left-retract-poses NIL)
	     ))
    

    (-> (member :right ?arm)
	;;TODO change name here nad start with one pose only

        (and
         (format "right one????")
         (lisp-fun man-int:get-action-trajectory :mixing
			:right ?grasp T ?object :context ?context :reso ?reso :tool-object-type ?source-type :container-arm :left
			?right-trajectory)
	    ;  (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grip-container
		;	?left-grip-container-poses)
	      (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :approach
			?right-approach-poses)
	       (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :start-mix
	       		?right-start-mix-poses)
	      (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :mid-mix
	      		?right-mid-mix-poses)
               (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :end-mix
                         ?right-end-mix-poses)
                (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :retract
                         ?right-retract-poses)

  
	      (lisp-fun get-arm-return :right ?right)
     	     (format "arm: ~a ~% ?right-trajectory: ~a" ?right ?right-trajectory))
	;;care u have to define the else here otherwise it cannot be resolved
	 (and (equal ?right NIL)
	      (equal ?right-trajectory NIL)
	      (equal ?right-approach-poses NIL)
	      (equal ?right-start-mix-poses NIL)
	      (equal ?right-mid-mix-poses NIL)
              (equal ?right-end-mix-poses NIL)
	      (equal ?right-retract-poses NIL)
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
                               (:source ?source-designator)
                               (:source-type ?source-type)

		               (:left-approach-poses ?left-approach-poses)
			       (:right-approach-poses ?right-approach-poses)
			       (:left-start-mix-poses ?left-start-mix-poses)
			       (:right-start-mix-poses ?right-start-mix-poses)
                               (:left-mid-mix-poses ?left-mid-mix-poses)
                               (:right-mid-mix-poses ?right-mid-mix-poses)
                               (:left-end-mix-poses ?left-end-mix-poses)
                               (:right-end-mix-poses ?right-end-mix-poses)
                               (:left-retract-poses ?left-retract-poses)
	                       (:right-retract-poses ?right-retract-poses)
			       )
                      ?resolved-action-designator)))

;;thats just for fun
(defun get-arm-return (?arm)
  ?arm)
  
;;   ;;;;;;;;;;;;;;;;;;
;;   ;grab-in-place
;;   ;;;;;;;;;;;;;;;;;
;; (def-fact-group grab-in-place (desig:action-grounding)

;;   (<- (desig:action-grounding ?action-designator (park-arms ?resolved-action-desig))
;;     (spec:property ?action-designator (:type :parking-arms))
;;     ;; get the arms list from the designator or infer it
;;     (once (or (spec:property ?action-designator (:arms ?arms-list))
;;               (-> (spec:property ?action-designator (:not-neck T))
;;                   (and (rob-int:robot ?robot-name)
;;                        (rob-int:arms-that-are-not-neck ?robot-name ?arms-list))
;;                   (and (rob-int:robot ?robot-name)
;;                        (rob-int:arms ?robot-name ?arms-list)))))
;;     ;; see if left arm and right arm are present
;;     ;; this is super non-general but has to be like this
;;     ;; because positioning-arm is so non-general
;;     (-> (member :left ?arms-list)
;;         (equal ?left-arm-p T)
;;         (equal ?left-arm-p NIL))
;;     (-> (member :right ?arms-list)
;;         (equal ?right-arm-p T)
;;         (equal ?right-arm-p NIL))
;;     (desig:designator :action ((:type :parking-arms)
;;                                (:left-arm ?left-arm-p)
;;                                (:right-arm ?right-arm-p))
;;                       ?resolved-action-desig))

;;   (<- (desig:action-grounding ?action-designator (perceive ?resolved-action-designator))
;;     (spec:property ?action-designator (:type :perceiving))
;;     ;;(spec:property ?action-designator (:counter ?_))
;;     ;; extract object from ?action-designator
;;     (spec:property ?action-designator (:object ?object-designator))
;;     (desig:current-designator ?object-designator ?current-object-desig)
;;     (spec:property ?current-object-desig (:type ?object-type))
;;     ;; test if the object designator has an name and check for occluduing obejcts if it has one 
;;     (-> (spec:property ?current-object-desig (:name ?object-name))
;;         (and (btr:bullet-world ?world)
;;              (cram-robot-interfaces:robot ?robot)
;;              (cram-robot-interfaces:camera-frame ?robot ?camera-frame)
;;              (btr:link-pose ?robot ?camera-frame ?camera-pose)
;;              (btr:occluding-objects ?world ?camera-pose ?object-name ?occluding-names)
;;              (-> (lisp-pred identity ?occluding-names)
;;                  (and (equal ?tmp (:occluding-names ?occluding-names))
;;                       (equal ?new-key (?tmp)))
;;                  (equal ?new-key NIL))
;;              (lisp-fun desig:extend-designator-properties ?action-designator ?new-key ?resolved-action-designator))
;;         (equal ?action-designator ?resolved-action-designator)))



;;   (<- (desig:action-grounding ?action-designator (grab-in-place ?resolved-action-designator))
;;     (spec:property ?action-designator (:type :grabbing-in-place))

;;     ;; extract info from ?action-designator
;;     (spec:property ?action-designator (:object ?object-designator))
;;     (desig:current-designator ?object-designator ?current-object-desig)
;;     (spec:property ?current-object-desig (:type ?object-type))
;;     (spec:property ?current-object-desig (:name ?object-name))

;;     (spec:property ?action-designator (:arm ?arm))
;;     (-> (lisp-type ?arm list)
;;         (equal ?arms ?arm)
;;         (equal ?arms (?arm)))
        
;;       ;; get the arm for grasping by checking if it is specified for ?object-type
;;     (man-int:arms-for-object-type ?object-type ?armss-for-object)
;;     (-> (equal ?armss-for-object nil)
;;         (-> (spec:property ?action-designator (:arm ?arms))
;;             (and (setof ?free-arm (man-int:robot-free-hand ?_ ?free-arm) ?free-arms)
;;                  (subset ?arms ?free-arms))
;;             (and (man-int:robot-free-hand ?_ ?free-arm)
;;                  (equal (?free-arm) ?arms)))
;;         (-> (spec:property ?action-designator (:arm ?arms))
;;             (and (setof ?free-arm (man-int:robot-free-hand ?_ ?free-arm) ?free-arms)
;;                  (subset ?arms ?free-arms)
;;                  (man-int:check-arms-for-object-type ?arms ?object-type))
;;             (and (setof ?free-arm (man-int:robot-free-hand ?_ ?free-arm) ?free-arms)
;;                  (man-int:check-arms-for-object-type ?free-arms ?object-type)
;;                  (equal ?arms ?free-arms))))
                 
;;     (lisp-fun man-int:get-object-transform ?current-object-desig ?object-transform)
;;     ;; infer missing information like ?grasp type, gripping ?maximum-effort, manipulation poses
;;     (lisp-fun man-int:calculate-object-faces ?object-transform (?facing-robot-face ?bottom-face))
;;     (-> (man-int:object-rotationally-symmetric ?object-type)
;;         (equal ?rotationally-symmetric t)
;;         (equal ?rotationally-symmetric nil))
;;     (lisp-fun man-int:get-action-gripping-effort ?object-type ?effort)
;;     (lisp-fun man-int:get-action-gripper-opening ?object-type ?gripper-opening)
;;     ;; get the type of the picking location, because the trajectory
;;     ;; might be different depending on the location type
;;     (once (or (and (spec:property ?current-object-desig (:location ?obj-loc))
;;                    (desig:current-designator ?obj-loc ?curr-obj-loc)
;;                    (man-int:location-reference-object ?curr-obj-loc ?obj-loc-obj)
;;                    (desig:current-designator ?obj-loc-obj ?curr-obj-loc-obj)
;;                    (spec:property ?curr-obj-loc-obj (:type ?location-type)))
;;               (equal ?location-type NIL)))

;;     ;; calculate trajectory with given grasps
;;     (lisp-fun man-int:get-action-grasps ?object-type ?arms ?object-transform ?grasps)    
;;     (equal ?objects (?current-object-desig))
;;     (-> (member :left ?arms)
;;         (and (-> (spec:property ?action-designator (:left-grasp ?left-grasp))
;;                  (true)
;;                  (member ?left-grasp ?grasps))
;;              (lisp-fun man-int:get-action-trajectory :grabbing-in-place :left
;;                        ?left-grasp ?location-type ?objects
;;                        ?left-trajectory)
;;              (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
;;                        ?left-reach-poses)
;;              (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :grasping
;;                        ?left-grasp-poses)
;; )
;;         (and (equal ?left-grasp NIL)
;;              (equal ?left-reach-poses NIL)
;;              (equal ?left-grasp-poses NIL)
;;              (equal ?left-lift-poses NIL)))

;;     (-> (member :right ?arms)
;;         (and  (-> (spec:property ?action-designator (:right-grasp ?right-grasp))
;;                   (true)
;;                   (member ?right-grasp ?grasps))
;;               (lisp-fun man-int:get-action-trajectory :grabbing-in-place
;;                         :right ?right-grasp ?location-type ?objects
;;                         ?right-trajectory)
;;              (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
;;                        ?right-reach-poses)
;;              (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grasping
;;                        ?right-grasp-poses)
;;              )
;;         (and (equal ?right-grasp NIL)
;;              (equal ?right-reach-poses NIL)
;;              (equal ?right-grasp-poses NIL)
;;              ))

;;     (once (or (lisp-pred identity ?left-trajectory)
;;               (lisp-pred identity ?right-trajectory)))

;;     (-> (lisp-pred identity ?left-grasp-poses)
;;         (equal ?left-grasp-poses (?look-pose . ?_))
;;         (equal ?right-grasp-poses (?look-pose . ?_)))

;;     (-> (man-int:robot-arm-is-also-a-neck ?robot ?arm)
;;         (equal ?robot-arm-is-also-a-neck T)
;;         (equal ?robot-arm-is-also-a-neck NIL))

;;     ;; put together resulting action designator
;;     (desig:designator :action ((:type :grabbing-in-place)
;;                                (:object ?current-object-desig)
;;                                (:arm ?arms)
;;                                (:gripper-opening ?gripper-opening)
;;                                (:effort ?effort)
;;                                (:left-grasp ?left-grasp)
;;                                (:right-grasp ?right-grasp)
;;                                (:location-type ?location-type)
;;                                (:look-pose ?look-pose)
;;                                (:robot-arm-is-also-a-neck ?robot-arm-is-also-a-neck)
;;                                (:left-reach-poses ?left-reach-poses)
;;                                (:right-reach-poses ?right-reach-poses)
;;                                (:left-grasp-poses ?left-grasp-poses)
;;                                (:right-grasp-poses ?right-grasp-poses))
;;                       ?resolved-action-designator))
  
