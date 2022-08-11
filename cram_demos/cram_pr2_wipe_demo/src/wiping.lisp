(in-package :pr2-wipe)

(defmethod get-object-type-robot-frame-wipe-approach-transform ((object-type (eql :colored-box)) arm (grasp (eql :countertop)))
  (print "pose is being called")
  (cl-transforms-stamped:make-transform-stamped "map" "base_footprint" 1
                                                (cl-tf:make-3d-vector 0.08 0.33 0.05) (cl-tf:make-quaternion 1 0 -1 0)))

(defmethod get-gripper-wipe-offset ()
  0.03
 )




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
  :grasp-translation `(0.0 0.06 ,0.020)
  :grasp-rot-matrix '((1  0  0)
                      (0 -1  0)
                      (0  0 -1))
  :pregrasp-offsets `(0.0 0.06 ,0.25)
  :2nd-pregrasp-offsets `(0.0 0.06 ,0.25)
  :lift-translation `(0.0 0.06 ,0.25)
  :2nd-lift-translation `(0.0 0.06 ,0.25))


