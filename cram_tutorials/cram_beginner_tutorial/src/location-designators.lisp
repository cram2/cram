(in-package :tut)

(defun navigation-goal-generator (designator)
  (declare (type location-designator designator))
  (with-desig-props (vertical-position horizontal-position) designator
    (let ((x-offset (ecase horizontal-position
                      (:left 0)
                      (:center (/ 11.0 3.0))
                      (:right (* (/ 11.0 3.0) 2))))
          (y-offset (ecase vertical-position
                      (:bottom 0)
                      (:center (/ 11.0 3.0))
                      (:top (* (/ 11.0 3.0) 2)))))
      (loop repeat 5
            collect (cl-transforms:make-3d-vector
                     (+ x-offset (random (/ 11.0 3.0)))
                     (+ y-offset (random (/ 11.0 3.0)))
                     0)))))

(register-location-generator
 5 navigation-goal-generator)

(defun navigation-goal-validator (designator solution)
  (declare (type location-designator designator))
  (when (and (desig-prop-value designator :vertical-position)
             (desig-prop-value designator :horizontal-position))
    (when (typep solution 'cl-transforms:3d-vector)
      (when
          (and
           (>= (cl-transforms:x solution) 0.5)
           (>= (cl-transforms:y solution) 0.5)
           (<= (cl-transforms:x solution) 10.5)
           (<= (cl-transforms:y solution) 10.5))
        :accept))))

(register-location-validation-function
 5 navigation-goal-validator)

;; (defun make-2d-pose-with-randomness (x y random-range)
;;   (cl-transforms:make-pose
;;    (cl-transforms:make-3d-vector (+ x (random random-range))
;;                                  (+ y (random random-range))
;;                                  0)
;;    (cl-transforms:make-identity-rotation)))

;; (def-fact-group location-designator-generator (desig-solution)
;;   (<- (position-offset :bottom 0))
;;   (<- (position-offset :top ?offset)
;;     (lisp-fun / 11.0 3.0 0.5 ?offset))

;;   (<- (position-offset :center ?offset)
;;     (lisp-fun / 11.0 3.0 ?offset))

;;   (<- (position-offset :left 0))
;;   (<- (position-offset :right ?offset)
;;     (position-offset :top ?offset))

;;   (<- (desig-solution ?desig ?solution)
;;     (loc-desig? ?desig)
;;     (desig-prop ?desig (:vertical-position ?vertical-position))
;;     (desig-prop ?desig (:horizontal-position ?horizontal-position))
;;     (position-offset ?vertical-position ?vertical-offset)
;;     (position-offset ?horizontal-position ?horizontal-offset)
;;     (lisp-fun / 11.0 3.0 ?random-range)
;;     (lisp-fun make-2d-pose-with-randomness
;;               ?horizontal-offset ?vertical-offset ?random-range ?solution)))
