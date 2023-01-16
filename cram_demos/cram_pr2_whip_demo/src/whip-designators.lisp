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

;temporary ..stay 
    (->( equal ?reso nil)
    (true))
    ;;======infer reso AHHHHHHH?! =====
  ;  (format "printout: ~a" (type-of ?action-designator))
  ;         (get-reso-from-list ?action-designator) ; (cdr (assoc ,:reso ,?action-designator :reso #'equal)))
  ;  (-> (spec:property ?action-designator (:reso ?reso))
   ;     (true)
    ;    (?reso is (get-action-reso ?reso))
     ;   )
;   (defmacro getassoc (key alist)
;  `(cdr (assoc ,key ,alist :test #'equal)))

;(defun get-object-transform (object-designator)
 ; (car (getassoc :transform (desig:desig-prop-value object-designator :pose))))

       ;(and  (format "reso was nil and set to: ~a" ?reso)
        ;    (get-action-reso :reso)
         ;   ))
          ;  (format "reso is prolly not nil ~a" ?reso)
       ; )
      ;  (lisp-fun get-action-reso ?reso))
         ;   (format "reso is:  ~a" ?reso))
      ;  (true))
      ; (equal ?reso 12)) ;default reso
    
	     ;TODO reso changes (and (lisp-fun get-default-reso ?reso)
	    
 
  ;      (true)
       ;)   
    
    ;;TODO: THIS STAYS THE SAME DONT CHANGE TILL HERE !!!! ++++++++++++

    
    ;; ==============  calculate trajectory with given grasp  ==============
    (equal ?objects (?current-object-desig))
    (-> (member :left ?arm)
	;;TODO change name here nad start with one pose only
	(and (lisp-fun man-int:get-action-trajectory :mixing
		       :left ?grasp T ?objects
		       ?left-trajectory) ;?reso)
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
	 (and (lisp-fun man-int:get-action-trajectory :mixing
			:right ?grasp T ?objects
			?right-trajectory) ;?reso)
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
                               (:left-end-mix-poses ?left-end-mix-poses)
                               (:right-end-mix-poses ?right-end-mix-poses)
                               (:left-retract-poses ?left-retract-poses)
	                       (:right-retract-poses ?right-retract-poses)
			       )
                      ?resolved-action-designator)))

;;thats just for fun
(defun get-arm-return (?arm)
  ?arm)

 (defun get-action-reso (?reso)
(if (eq ?reso nil) 12 ?reso))

(defun get-reso-from-list (alist)
 (cdr (assoc :reso alist)))
