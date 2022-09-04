(in-package :pr2-wipe)

(defun wipe (&key
               ((:arm ?arm))
               ((:surface ?surface))
               ((:collision-mode ?collision-mode))
               ((:surface-type ?surface-type))
               ((:pose ?pose))
               ((:left-wipe-poses ?left-wipe-poses))
               ((:left-approach-poses ?left-approach-poses))
               ((:right-wipe-poses ?right-wipe-poses))
               ((:right-approach-poses ?right-approach-poses))
             &allow-other-keys)
  ;; (print ?arm)
  ;; (print ?surface)
  ;; (print ?collision-mode)
  ;; (print ?surface-type)
  ;; (print ?pose)
  ;; (print ?left-wipe-poses)
  (print ?left-wipe-poses)
  (exe:perform
                (desig:an action
                          (type approaching)
                          (left-poses ?left-approach-poses)
                          (desig:when ?collision-mode
                            (collision-mode ?collision-mode))))

  (print `(,(last ?left-wipe-poses)))
  (sleep 1.0)

  (let ((?current-left-wipe-poses `(,(pop ?left-wipe-poses))))
   (exe:perform
                (desig:an action
                          (type approaching)
                          (left-poses ?current-left-wipe-poses)
                          (desig:when ?collision-mode
                            (collision-mode ?collision-mode)))))
  (sleep 0.1)

  (loop while ?left-wipe-poses

  
        do

           (let ((?current-left-wipe-poses `(,(pop ?left-wipe-poses))))
             (exe:perform
              (desig:an action
                        (type approaching)
                        (left-poses ?current-left-wipe-poses)
                        (desig:when ?collision-mode
                          (collision-mode ?collision-mode)))))
          (sleep 0.1)
  ))


(def-fact-group wipe-actions (desig:action-grounding)
  
  (<- (desig:action-grounding ?action-designator (wipe ?resolved-action-designator))
    (spec:property ?action-designator (:type :wiping))
    (spec:property ?action-designator (:grasp ?grasp))

    (-> (desig:desig-prop ?action-designator (:surface ?surface-designator))
        (true)
        (equalp ?surface-designator nil))

  
     
     (desig:current-designator ?surface-designator ?current-surface-designator)
     (spec:property ?current-surface-designator (:type ?surface-type))



    ;;Ausgewählter arm muss immer der arm mit dem schwamm sein/harte übergabe
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (man-int:robot-free-hand ?_ ?arm))


    (equal ?a (?current-surface-designator))

    ;;hier fehlt noch dynamic grasp/ dynamische typerkennung ???
    ;;typerkennung über object type nicht möglich da alle vom type colored-box sind
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory :wiping ?arm ?grasp T ?a ?pose)
             (lisp-fun man-int:get-traj-poses-by-label ?pose :wiping
                       ?left-wipe-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?pose :approach
                       ?left-approach-poses))
        (and (equal ?left-wipe-poses NIL)
             (equal ?left-approach-poses NIL)))

    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory :wiping ?arm ?grasp T ?a ?pose)
             (lisp-fun man-int:get-traj-poses-by-label ?pose :wiping
                       ?right-wipe-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?pose :approach
                       ?right-approach-poses))
        (and (equal ?right-wipe-poses NIL)
             (equal ?right-approach-poses NIL)))

    (-> (desig:desig-prop ?action-designator (:collision-mode ?collision-mode))
        (true)
        (equal ?collision-mode NIL))

   
    
    

    (desig:designator :action ((:type :wiping)
                               (:arm ?arm)
                               (:surface ?current-surface-designator)
                               (:surface-type ?surface-type)
                               (:collision-mode ?collision-mode)
                               (:pose ?pose)
                               (:left-wipe-poses ?left-wipe-poses)
                               (:right-wipe-poses ?right-wipe-poses)
                               (:left-approach-poses ?left-approach-poses)
                               (:right-approach-poses ?right-approach-poses)
                               )
                      ?resolved-action-designator)))
