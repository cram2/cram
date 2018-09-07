(in-package :kvr)

;; Incomplete and experimental code for implementing gaussians
(defvar *gaussian-poses-list* nil)


(defun calculate-pose-covariance ()
  (location-costmap:2d-pose-covariance *gaussian-poses-list*))

;; def gaussian costmap
 (prolog:def-fact-group left-of-rules (location-costmap:desig-costmap)
  (prolog:<- (location-costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:behind ?pose))
    (prolog:lisp-fun cl-transforms:origin ?pose ?pose-origin)
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function
     behind-cost-function
     (location-costmap:make-gauss-cost-function ?pose-origin
                                                (prolog:lisp-fun (second (calculate-pose-covariance))))
     ?costmap)))





