
(in-package :kipla-reasoning)

(defclass manipulaton-metadata ()
  ((obj-location :initarg :obj-location :accessor obj-location)
   (obj-type :initarg :obj-type :accessor obj-type)
   (location :initarg :location :accessor location)
   (status :initarg :success :accessor status)))

(defclass pick-and-plance-metadata ()
  ((pick-up-metadata :initarg :pick-up-metadata
                     :accessor pick-up-metadata)
   (put-down-metadata :initarg :put-down-metadata
                      :accessor put-down-metadata)))

(def-fact-group pick-and-place-analysis ()
  (<- (pick-up-obj-desig ?tsk ?obj-desig)
    (task-goal ?tsk (achieve (object-in-hand ?obj-desig ?_))))

  (<- (obj-desig-type ?desig ?type)
    (or (desig-prop ?desig (type ?type))
        (desig-prop ?desig (kipla-reasoning::shape-model ?type))
        (desig-prop ?desig (kipla-reasoning::segment-prototype ?type))))

  (<- (obj-desig-loc ?desig ?loc)
    (desig-prop ?desig (at ?loc)))

  (<- (put-down-location ?loc-desig)
    ;; The location the robot stands at while trying to put down an
    ;; object
    (task ?tsk)
    (task-goal ?tsk (achieve (arms-at ?action-desig)))
    (desig-prop ?action-desig (to put-down))
    (task-location-context ?tsk ?loc-desig))

  (<- (pick-up-metadata (?tsk ?type ?obj ?obj-loc ?outcome))
    (task-goal ?tsk (achieve (object-in-hand ?_ ?_)))
    (task-goal ?reach-tsk (achieve (arms-at ?traj)))
    (desig-prop ?traj (to grasp))
    (desig-prop ?traj (obj ?obj))
    (obj-desig-type ?obj ?type)
    (obj-desig-loc ?obj ?obj-loc)
    (task-location-context ?reach-tsk ?pick-up-loc)
    (task-outcome ?reach-tsk ?outcome))

  (<- (put-down-metadata (?tsk ?type ?obj ?destination ?outcome))
    (task ?tsk)
    (task-goal ?tsk (achieve (object-placed-at ?_ ?dest)))
    (task-goal ?put-down-tsk (achieve (arms-at ?traj)))
    (desig-prop ?traj (to put-down))
    (desig-prop ?traj (obj ?obj))
    (obj-desig-type ?obj ?type)
    (lisp-fun current-desig ?dest ?destination)
    (task-outcome ?put-down-tsk ?outcome)
    (task-location-context ?put-down-tsk ?put-down-loc)))
