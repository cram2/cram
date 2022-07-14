(in-package :pr2-wipe)


(def-fact-group wipe-plans (desig:action-grounding)
  
  (<- (desig:action-grounding ?action-designator (wipe ?resolved-action-desig))
    (spec:property ?action-designator (:type :wiping))
    
    






        (desig:designator :action ((:type :wiping)
                                   (:surface ?current-surface-desig)
                                   (:surface-name ?surface-name)
                                   (:arm ?arm)
                                   (:left-wipe-poses ?left-wipe-poses)
                                   (:right-wipe-poses ?right-wipe-poses)
                                   (:collision-mode ?collision-mode))
                          ?resolved-action-designator))
