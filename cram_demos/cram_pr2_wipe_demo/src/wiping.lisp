(in-package :pr2-wipe)


;; (defmethod get-object-type-robot-frame-wipe-approach-transform ((object-type (eql :colored-box)) arm (grasp (eql :countertop)))
;;   (print "pose is being called")
;;   (cl-transforms-stamped:make-transform-stamped "map" "base_footprint" 0
;;                                                 (cl-tf:make-3d-vector 0.1 0.3 0.05) (cl-tf:make-quaternion 1 0 -1 0)))
   

;; (defmethod get-object-type-robot-frame-wipe-approach-transform ((object-type (eql :colored-box)) arm (grasp (eql :vertical)))
;;   (print "pose is being called")
;;   (cl-transforms-stamped:make-transform-stamped "map" "base_footprint" 1
;;                                                 (cl-tf:make-3d-vector 0.1 0.36 0.05) (cl-tf:make-quaternion 1 0 -1 0)))


(defmethod get-object-type-robot-frame-wipe-approach-transform-generic (surface (object-type (eql :colored-box)) arm (grasp (eql :countertop)))
  (print "pose is being called")

  (let* (
          (x (cl-transforms:x
                               (cl-bullet::bounding-box-dimensions
                                (btr::aabb  (btr:object btr:*current-bullet-world*  (desig:desig-prop-value surface :name))))))
         (y (cl-transforms:y
                               (cl-bullet::bounding-box-dimensions
                                (btr::aabb  (btr:object btr:*current-bullet-world* (desig:desig-prop-value surface :name)))))))
  
  (cl-transforms-stamped:make-transform-stamped "map" "base_footprint" 0
                                                (cl-tf:make-3d-vector (/ x 2) (/ y 2) 0.01) (cl-tf:make-quaternion 1 0 -1 0))))

(defmethod get-object-type-robot-frame-wipe-approach-transform-generic (surface (object-type (eql :colored-box)) arm (grasp (eql :vertical)))
  (print "pose is being called")

  (let* (
         (y (cl-transforms:y
                               (cl-bullet::bounding-box-dimensions
                                (btr::aabb  (btr:object btr:*current-bullet-world* (desig:desig-prop-value surface :name))))))
          (z (cl-transforms:z
                               (cl-bullet::bounding-box-dimensions
                                (btr::aabb  (btr:object btr:*current-bullet-world* (desig:desig-prop-value surface :name)))))))
  
  (cl-transforms-stamped:make-transform-stamped "map" "base_footprint" 0
                                                (cl-tf:make-3d-vector -0.01 (/ y 2) (/ z 2)) (cl-transforms:axis-angle->quaternion
                                             (cl-transforms:make-3d-vector 0 0 1)
                                             0.00))))



(defmethod get-object-type-robot-frame-wipe-approach-transform-generic (surface (object-type (eql :colored-box)) arm (grasp (eql :knife)))
  (print "pose is being called")

  (let* (
          (x (cl-transforms:x
                               (cl-bullet::bounding-box-dimensions
                                (btr::aabb  (btr:object btr:*current-bullet-world*  (desig:desig-prop-value surface :name))))))
         (y (cl-transforms:y
                               (cl-bullet::bounding-box-dimensions
                                (btr::aabb  (btr:object btr:*current-bullet-world* (desig:desig-prop-value surface :name)))))))
  
  (cl-transforms-stamped:make-transform-stamped "map" "base_footprint" 0
                                                (cl-tf:make-3d-vector (/ x 2) (/ y 2) 0.01) (cl-transforms:axis-angle->quaternion
    (cl-tf:make-3d-vector 0 1 0)
    2.1))))





;; (defun make-transform-stamped (frame-id child-frame-id stamp translation rotation)
;;   (make-instance 'transform-stamped
;;     :frame-id frame-id
;;     :child-frame-id child-frame-id
;;     :stamp stamp
;;     :translation translation
;;     :rotation rotation))

  ;; (print approach-pose)
  ;;   (let* ((x-dimensions
  ;;                     (cl-transforms:x
  ;;                      (cl-bullet::bounding-box-dimensions
  ;;                       (btr::aabb  (btr:object btr:*current-bullet-world* 'surface-1))))))


  ;;             (print x-dimensions)
  ;;           )





;; countertop
(man-int:def-object-type-to-gripper-transforms '(:colored-box)
    '(:left :right) :countertop
  :grasp-translation `(0.0 0.00 ,0.0)
  :grasp-rot-matrix '((0  0  0)
                      (0 0  0)
                      (0  0 0))
  :pregrasp-offsets `(0.0 0.0 ,0.0)
  :2nd-pregrasp-offsets `(0.0 0.0 ,0.0)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

;;vertical
(man-int:def-object-type-to-gripper-transforms '(:colored-box)
    '(:left :right) :vertical
  :grasp-translation `(0.0 0.00 ,0.0)
  :grasp-rot-matrix '((0  0  0)
                      (0 0  0)
                      (0  0 0))
  :pregrasp-offsets `(0.0 0.0 ,0.0)
  :2nd-pregrasp-offsets `(0.0 0.0 ,0.0)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

;;knife
(man-int:def-object-type-to-gripper-transforms '(:colored-box)
    '(:left :right) :knife
  :grasp-translation `(-0.112 -0.00 ,0.195)
  :grasp-rot-matrix '((0  0  0)
                      (0 0 0)
                      (0  0 0))
  :pregrasp-offsets `(0.0 0.00 ,0.0)
  :2nd-pregrasp-offsets `(0.0 0.0 ,0.0)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))


;;(-0.11 -0.00 ,0.18)
;;(-0.106 -0.00 ,0.205)
