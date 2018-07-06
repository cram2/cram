(in-package :le)

;; Incomplete and experimental code for implementing gaussians
(defvar *gaussian-poses-list* nil)

;; nr-poses: number of poses one wants to be read out from the data
(defun get-all-base-poses (nr-poses)
  ;; make sure to start with a clean slate:
  (get-grasp-something-poses)  ;;; reads out data from OE
  (setq *gaussian-poses-list* nil)
  (dotimes (c nr-poses )
    (get-next-obj-poses c)
    (push (cl-tf:pose-stamped->pose (set-grasp-base-pose (apply-bullet-transform  (make-poses "?PoseCameraStart")))) *gaussian-poses-list*)
  ))


(defun calculate-pose-covariance ()
  (location-costmap:2d-pose-covariance *gaussian-poses-list*))


;; note: costmap is already being defined in the initialization of bullet-world
;; this is copied from bullet-world.lisp




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





