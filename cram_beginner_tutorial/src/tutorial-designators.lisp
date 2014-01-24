(in-package :tut)

(defstruct turtle-pose
  "represents a position of a turtle"
  x
  y)

(defun navigation-goal-generator (desig)
  (let ((x (- 1)) (y (- 1)))
    (dolist (elem (description desig))
      (case
          (car elem)
        (vpos
         (case (cadr elem)
           (center (setq y 6))
           (top (setq y 11))
           (bottom (setq y 1))))
        (hpos
         (case (cadr elem)
           (center (setq x 6))
           (right (setq x 11))
           (left (setq x 1))))))
    (list
     (tf:make-pose-stamped
      "turtlesim"
      (roslisp:ros-time)
      (cl-transforms:make-3d-vector x y 0)
      (cl-transforms:make-quaternion 0 0 0 1)))))

(defun turtle-pose-validator (desig pose)
  (declare (ignore desig))
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

(def-fact-group navigation-action-designator (action-desig)
  (<- (action-desig ?designator (navigation ?goal))
    (desig-prop ?designator (type navigation))
    (desig-prop ?designator (goal ?goal))))

(cram-process-modules:def-process-module turtle-navigation (action-designator)
  (roslisp:ros-info (turtle-process-modules)
                    "Turtle navigation invoked with action designator `~a'."
                    action-designator)
  (destructuring-bind (cmd action-goal) (reference action-designator)
    (ecase cmd
      (navigation
       (print (cl-transforms:origin (reference action-goal)))
       (move-to (cl-transforms:origin (reference action-goal)))))))

(def-fact-group turtle-navigation (matching-process-module
                                   available-process-module)

  (<- (matching-process-module ?designator turtle-navigation)
    (or (desig-prop ?designator (type navigation))))

  (<- (available-process-module turtle-navigation)
    (symbol-value cram-projection:*projection-environment* nil)))