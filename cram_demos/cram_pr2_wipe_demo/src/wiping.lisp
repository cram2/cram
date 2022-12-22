(in-package :pr2-wipe)


;;Function that creates a transform by extracting the x,y and z values of a given surface. Differentiates by grasp type :spreading, :vertical and :horizontal.
(defmethod get-object-type-robot-frame-wipe-approach-transform-generic (surface (object-type (eql :colored-box)) arm grasp)

  (case grasp
    (:spreading
       (let* ((x (cl-transforms:x (cl-bullet::bounding-box-dimensions
                               (btr::aabb
                                (btr:object btr:*current-bullet-world*
                                            (desig:desig-prop-value surface :name))))))
         (y (cl-transforms:y (cl-bullet::bounding-box-dimensions
                              (btr::aabb
                               (btr:object btr:*current-bullet-world*
                                           (desig:desig-prop-value surface :name)))))))
  
         (cl-transforms-stamped:make-transform-stamped "base_footprint" "base_footprint" 0.0
                                                       (cl-tf:make-3d-vector (/ x 2) (/ y 2) 0.01) (cl-transforms:axis-angle->quaternion
    (cl-tf:make-3d-vector 0 1 0)
    2.1))))


    (:vertical
      (let* ((y (cl-transforms:y (cl-bullet::bounding-box-dimensions
                              (btr::aabb
                               (btr:object btr:*current-bullet-world*
                                           (desig:desig-prop-value surface :name))))))
          (z (cl-transforms:z (cl-bullet::bounding-box-dimensions
                               (btr::aabb
                                (btr:object btr:*current-bullet-world*
                                            (desig:desig-prop-value surface :name)))))))
  
        (cl-transforms-stamped:make-transform-stamped "base_footprint" "base_footprint" 0.0
                                                      (cl-tf:make-3d-vector -0.01 (/ y 2) (/ z 2)) (cl-tf:make-identity-rotation))))



    (:horizontal
     (let* ((x (cl-transforms:x (cl-bullet::bounding-box-dimensions
                                 (btr::aabb
                                (btr:object btr:*current-bullet-world*
                                            (desig:desig-prop-value surface :name))))))
         (y (cl-transforms:y (cl-bullet::bounding-box-dimensions
                              (btr::aabb
                               (btr:object btr:*current-bullet-world*
                                           (desig:desig-prop-value surface :name)))))))
  
         (cl-transforms-stamped:make-transform-stamped "base_footprint" "base_footprint" 0.0
                                                       (cl-tf:make-3d-vector (/ x 2) (/ y 2) 0.01) (cl-tf:make-quaternion 1 0 -1 0))))))



;; horizontal
(man-int:def-object-type-to-gripper-transforms '(:colored-box)
    '(:left :right) :horizontal
  :grasp-translation `(0.0 0.00 ,0.0)
  :grasp-rot-matrix '((0  0  0)
                      (0 0  0)
                      (0  0 0))
  :pregrasp-offsets `(0.01 0.01 ,0.01)
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
  :pregrasp-offsets `(0.0 1 0.01 ,0.01)
  :2nd-pregrasp-offsets `(0.0 0.0 ,0.0)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

;;knife
(man-int:def-object-type-to-gripper-transforms '(:colored-box)
    '(:left) :spreading
  :grasp-translation `(-0.112 -0.00 ,0.195)
  :grasp-rot-matrix '((0 0 0)
                      (0 0 0)
                      (0  0 0))
  :pregrasp-offsets `(0.1 0.001 ,0.01)
  :2nd-pregrasp-offsets `(0.0 0.0 ,0.0)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))

(man-int:def-object-type-to-gripper-transforms '(:colored-box)
    '(:right) :spreading
  :grasp-translation `(-0.112 -0.00 ,0.058)
  :grasp-rot-matrix '((0 0 0)
                      (0 0 0)
                      (0  0 0))
  :pregrasp-offsets `(0.0 0.00 ,0.0)
  :2nd-pregrasp-offsets `(0.0 0.0 ,0.0)
  :lift-translation `(0.0 0.0 ,0.0)
  :2nd-lift-translation `(0.0 0.0 ,0.0))





;;left
;;(-0.112 -0.00 ,0.195)
;;right
;;(-0.112 -0.00 ,0.058)
