(in-package :pr2-nav-pm)

(def-fact-group pr2-navigation-designators (action-desig)
  
    (<- (action-desig ?desig ?goal-location)
      ;; Goal based action navigation
      (desig-prop ?desig (type navigation))
      (desig-prop ?desig (goal ?goal-location))))