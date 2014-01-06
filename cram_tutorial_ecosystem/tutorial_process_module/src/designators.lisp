(in-package :tutorial-process-module)

(def-fact-group tutorial-manipulation-designators (action-desig)

  (<- (action-desig ?desig (perceive ?obj))
    (desig-prop ?desig (to perceive))
    (desig-prop ?desig (obj ?obj)))
  
  (<- (action-desig ?desig (ground ?obj))
    (desig-prop ?desig (to ground))
    (desig-prop ?desig (obj ?obj)))
  
  (<- (action-desig ?desig (move ?obj ?pose))
    (desig-prop ?desig (to move))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (loc ?loc))
    (desig-prop ?loc (pose ?pose))))

(def-fact-group tutorial-process-module (matching-process-module
					 available-process-module)

  (<- (matching-process-module ?designator tutorial-process-module)
    (or (desig-prop ?designator (to perceive))
        (desig-prop ?designator (to move))
        (desig-prop ?designator (to ground))))

  (<- (available-process-module tutorial-process-module)
    (symbol-value cram-projection:*projection-environment* nil)))
