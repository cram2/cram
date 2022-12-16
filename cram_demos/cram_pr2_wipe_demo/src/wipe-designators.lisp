(in-package :pr2-wipe)

(defun wipe (&key
               ((:collision-mode ?collision-mode))
               ((:left-wipe-poses ?left-wipe-poses))
               ((:left-approach-pose ?left-approach-pose))
               ((:right-wipe-poses ?right-wipe-poses))
               ((:right-approach-pose ?right-approach-pose))
             &allow-other-keys)



  ;;===========================================Approach-pose========================================

  
  (mapc
   (lambda (?current-left-approach-pose)
     (let ((?pose `(,?current-left-approach-pose)))
       (exe:perform
        (desig:an action
                  (type wipe-increment)
                  (left-poses ?pose)
                  (desig:when ?collision-mode
                    (collision-mode ?collision-mode))))))
   ?left-approach-pose)


  (mapc
   (lambda (?current-right-approach-pose)
     (let ((?pose `(,?current-right-approach-pose)))
       (exe:perform
        (desig:an action
                  (type wipe-increment)
                  (right-poses ?pose)
                  (desig:when ?collision-mode
                    (collision-mode ?collision-mode))))))
   ?right-approach-pose)



  (sleep 0.1)


;;=======================================Wipe-poses===============================================


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
        (and (lisp-fun man-int:get-action-trajectory :wiping ?arm ?current-grasp T ?surface ?pose)
             (lisp-fun man-int:get-traj-poses-by-label ?pose :wiping
                       ?left-wipe-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?pose :approach
                       ?left-approach-pose))
        (and (equal ?left-wipe-poses NIL)
             (equal ?left-approach-pose NIL)))

    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory :wiping ?arm ?current-grasp T ?surface ?pose)
             (lisp-fun man-int:get-traj-poses-by-label ?pose :wiping
                       ?right-wipe-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?pose :approach
                       ?right-approach-pose))
        (and (equal ?right-wipe-poses NIL)
             (equal ?right-approach-pose NIL)))


    (desig:designator :action ((:type :wiping)
                               (:collision-mode ?collision-mode)
                               (:left-wipe-poses ?left-wipe-poses)
                               (:right-wipe-poses ?right-wipe-poses)
                               (:left-approach-pose ?left-approach-pose)
                               (:right-approach-pose ?right-approach-pose)
                               )
                      ?resolved-action-designator)))


