(in-package :pr2-em)

;;; DRAWER COSTMAP

(defun make-poses-cost-function (poses)
  "`poses' are the poses according to which the relation is resolved."
  (let ((meancovs (location-costmap:2d-pose-covariance poses 0.05)))
    (location-costmap:make-gauss-cost-function (first meancovs)
                                               (second meancovs))))

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'poses-cost-function))) 10)

(def-fact-group environment-manipulation-costmap (location-costmap:desig-costmap)
  (<- (location-costmap:desig-costmap ?designator ?costmap)
    ;;(or (cram-robot-interfaces:visibility-designator ?desig)
    ;;    (cram-robot-interfaces:reachability-designator ?desig))
    (desig:desig-prop ?designator (:poses ?poses))
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function
     poses-cost-function
     (make-poses-cost-function ?poses)
     ?costmap)
    )
  )
