(in-package :kvr)

(defun human-to-robot-hand-transform ()
  "Defines the offset between the human hand from the virtual reality to the
robot standart gripper, which has been calculated manually.
RETURNS: a cl-transform."
  (let ((alpha  0)) ; (/ pi 4)
    (cl-transforms:make-transform
     (cl-transforms:make-3d-vector 0.0 -0.07 0.2)
     (cl-transforms:matrix->quaternion
      (make-array '(3 3)
                  :initial-contents
                  `((0                1 0)
                    (,(- (cos alpha)) 0 ,(- (sin alpha)))
                    (,(- (sin alpha)) 0 ,(cos alpha))))))))

;; General Grasp
;; Name of type string or use filter
(defmethod get-object-type-to-gripper-transform (object-type
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))

  (let* ((transf
           (cl-tf:transform*
            (cl-tf:transform-inv
             (get-object-location-at-start-by-object-type
              (roslisp-utilities:rosify-lisp-name
               (object-type-fixer object-name))))
            (get-hand-location-at-start-by-object-type
             (roslisp-utilities:rosify-lisp-name
              (object-type-fixer object-name)))
            (human-to-robot-hand-transform)))
         (end-transf
           (cl-tf:transform->transform-stamped
            (roslisp-utilities:rosify-underscores-lisp-name object-name)
            (ecase arm
              (:left cram-tf:*robot-left-tool-frame*)
              (:right cram-tf:*robot-right-tool-frame*))
            0.0
            transf)))
    end-transf))

(defmethod get-object-type-to-gripper-pregrasp-transform (object-type
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped
   grasp-transform
   :x-offset (- cram-knowrob-pick-place::*cereal-pregrasp-xy-offset*)
   :z-offset cram-knowrob-pick-place::*lift-z-offset*))

(defmethod get-object-type-to-gripper-2nd-pregrasp-transform (object-type
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped
   grasp-transform
   :x-offset (- cram-knowrob-pick-place::*cereal-pregrasp-xy-offset*)))

(defmethod get-object-type-to-gripper-lift-transform (object-type
                                                      object-name
                                                      arm
                                                      (grasp (eql :human-grasp))
                                                      grasp-transform)
  (cram-tf:translate-transform-stamped
   grasp-transform
   :z-offset cram-knowrob-pick-place::*lift-z-offset*))

(defmethod get-object-type-to-gripper-2nd-lift-transform (object-type
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-pose)
  (cram-tf:translate-transform-stamped
   grasp-pose :x-offset 0.0))

