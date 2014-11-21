(in-package :tut)

(defun navigation-goal-generator (location-designator)
  (let ((retq ()))
      (dotimes (k 10)
          (let ((x (- 1)) (y (- 1)))
              (dolist (elem (description location-designator))
                  (case
                      (car elem)
                          (vpos
                              (case (cadr elem)
                                  (center (setq y (+ 6 (random 4.0) -2)) )
                                  (top (setq y (+ 11 (random 4.0) -2)) )
                                  (bottom (setq y (+ 1 (random 4.0) -2)))))
                          (hpos
                              (case (cadr elem)
                                  (center (setq x (+ 6 (random 4.0) -2)))
                                  (right (setq x (+ 11 (random 4.0) -2)))
                                  (left (setq x (+ 1 (random 4.0) -2)))))))
    (setq retq (append retq
         (list (cl-tf:make-pose-stamped
              "turtlesim"
              (roslisp:ros-time)
              (cl-transforms:make-3d-vector x y 0)
              (cl-transforms:make-quaternion 0 0 0 1)))))))
    (return-from navigation-goal-generator retq)))

(defun turtle-pose-validator (location-designator pose)
  (declare (ignore location-designator))
  (when (typep pose 'cl-transforms:pose)
    (if
     (or
      (< (cl-transforms:x
          (cl-transforms:origin pose))
         0)
      (< (cl-transforms:y
          (cl-transforms:origin pose))
         0)
      (> (cl-transforms:x
          (cl-transforms:origin pose))
         11.8)
      (> (cl-transforms:y
          (cl-transforms:origin pose))
         11.8))
     (return-from turtle-pose-validator :reject)
     (return-from turtle-pose-validator :accept))))

(register-location-generator
 15 navigation-goal-generator)

(register-location-validation-function
 15 turtle-pose-validator)

