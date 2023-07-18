(in-package :pr2-wipe)

;;@author Felix Krause
(defun wipe (&key
               ((:collision-mode ?collision-mode))
               ((:left-wipe-poses ?left-wipe-poses))
               ((:left-initial-pose ?left-initial-pose))
               ((:right-wipe-poses ?right-wipe-poses))
               ((:right-initial-pose ?right-initial-pose))
             &allow-other-keys)



;;===========================================Initial-pose========================================

  ;;Executes the initial pose of either the left or right arm.
  (mapc
   (lambda (?current-left-initial-pose)
     (let ((?pose `(,?current-left-initial-pose)))
       (exe:perform
        (desig:an action
                  (type wipe-increment)
                  (left-poses ?pose)
                  (desig:when ?collision-mode
                    (collision-mode ?collision-mode))))))
  
   ?left-initial-pose)


  (mapc
   (lambda (?current-right-initial-pose)
     (let ((?pose `(,?current-right-initial-pose)))
       (exe:perform
        (desig:an action
                  (type wipe-increment)
                  (right-poses ?pose)
                  (desig:when ?collision-mode
                    (collision-mode ?collision-mode))))))
   ?right-initial-pose)



  (sleep 0.1)


;;=======================================Wipe-poses===============================================

  ;;Executes the wiping poses of either the left or right arm.
  (mapc
   (lambda (?current-left-wipe-poses)
     (let ((?poses `(,?current-left-wipe-poses)))
       (exe:perform
        (desig:an action
                  (type wipe-increment)
                  (left-poses ?poses)
                  (desig:when ?collision-mode
                    (collision-mode ?collision-mode))))))
   ?left-wipe-poses)


  (mapc
   (lambda (?current-right-wipe-poses)
     (let ((?poses `(,?current-right-wipe-poses)))
       (exe:perform
        (desig:an action
                  (type wipe-increment)
                  (right-poses ?poses)
                  (desig:when ?collision-mode
                    (collision-mode ?collision-mode))))))
   ?right-wipe-poses))

 

;;@author Felix Krause
(def-fact-group wipe-actions (desig:action-grounding)
  
  (<- (desig:action-grounding ?action-designator (wipe ?resolved-action-designator))

    ;;Extract properties from the Designator.
    (spec:property ?action-designator (:type :wiping))
    (spec:property ?action-designator (:grasp ?grasp))
    (spec:property ?action-designator (:arm ?arm))
    (desig:desig-prop ?action-designator (:surface ?surface-designator))

    ;;Extracts the surface-type out of the surface-designator that is contained in the action Designator.
    (desig:current-designator ?surface-designator ?current-surface-designator)
    (spec:property ?current-surface-designator (:type ?surface-type))
    (equal ?surface (?current-surface-designator))

    ;;Infer the grasp type.
    (-> (equal ?grasp :scrubbing)
        (lisp-fun differentiate-surface-types ?grasp ?surface ?current-grasp)
        (equal ?grasp ?current-grasp))
    
    ;;Extract the collision-mode.
    (desig:desig-prop ?action-designator (:collision-mode ?collision-mode))
 
    ;;Calls the trajectory-calculation depending on what arm the Designator is being called with. Saves the resulting poses in ?left-wipe/approach-poses or ?right-wipe/approach-poses.
    (-> (equal ?arm :left)
        (and (lisp-fun get-trajectory :wiping ?arm ?current-grasp T ?surface ?lists)
             (lisp-fun man-int:get-traj-poses-by-label ?lists :wiping
                       ?left-wipe-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?lists :initial
                       ?left-initial-pose))
        (and (equal ?left-wipe-poses NIL)
             (equal ?left-initial-pose NIL)))

    (-> (equal ?arm :right)
        (and (lisp-fun get-trajectory :wiping ?arm ?current-grasp T ?surface ?lists)
             (lisp-fun man-int:get-traj-poses-by-label ?lists :wiping
                       ?right-wipe-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?lists :initial
                       ?right-initial-pose))
        (and (equal ?right-wipe-poses NIL)
             (equal ?right-initial-pose NIL)))

    ;;Puts together resulting action Designator.
    (desig:designator :action ((:type :wiping)
                               (:collision-mode ?collision-mode)
                               (:left-wipe-poses ?left-wipe-poses)
                               (:right-wipe-poses ?right-wipe-poses)
                               (:left-initial-pose ?left-initial-pose)
                               (:right-initial-pose ?right-initial-pose))
                      ?resolved-action-designator))


;;Atomic Wipe-Increment Designator that is based on the atomic Approach Designator.
(<- (desig:action-grounding ?action-designator (pp-plans::move-arms-in-sequence
                                                  ?resolved-action-designator))
    
    (spec:property ?action-designator (:type :wipe-increment))
    (or (spec:property ?action-designator (:left-poses ?left-poses))
              (equal ?left-poses nil))
    (or (spec:property ?action-designator (:right-poses ?right-poses))
              (equal ?right-poses nil))
    (or (spec:property ?action-designator (:collision-mode ?collision))
              (equal ?collision :allow-all))
  (pp-plans::infer-motion-flags ?action-designator ?_ ?move-base ?align-planes-left ?align-planes-right)
  (desig:designator :action ((:type :wipe-increment)
                             (:left-poses ?left-poses)
                             (:right-poses ?right-poses)
                             (:collision-mode ?collision)
                             (:move-base ?move-base)
                             (:align-planes-left ?align-planes-left)
                             (:align-planes-right ?align-planes-right))
                    ?resolved-action-designator)))
