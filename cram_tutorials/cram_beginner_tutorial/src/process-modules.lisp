(in-package :tut)

(def-process-module actionlib-navigation (action-designator)
  (roslisp:ros-info (turtle-process-modules)
                    "Turtle shape navigation invoked with action designator `~a'."
                    action-designator)
  (destructuring-bind (command action-goal) (reference action-designator)
    (ecase command
      (draw-shape
       (call-shape-action
        :edges (turtle-shape-edges action-goal)
        :radius (turtle-shape-radius action-goal))))))

(def-process-module simple-navigation (action-designator)
  (roslisp:ros-info (turtle-process-modules)
                    "Turtle simple navigation invoked with action designator `~a'."
                    action-designator)
  (destructuring-bind (command action-goal) (reference action-designator)
    (ecase command
      (go-to
       (when (typep action-goal 'location-designator)
         (let ((target-point (reference action-goal)))
           (roslisp:ros-info (turtle-process-modules)
                             "Going to point ~a." target-point)
           (move-to target-point)))))))

(defmacro with-turtle-process-modules (&body body)
  `(with-process-modules-running
       (actionlib-navigation
        simple-navigation)
     ,@body))

(defun draw-hexagon (radius)
  (let ((turtle-name "turtle1"))
    (start-ros-node turtle-name)
    (init-ros-turtle turtle-name)
    (top-level
      (with-turtle-process-modules
        (process-module-alias :navigation 'actionlib-navigation)
        (with-designators
            ((trajectory :action `((:type :shape) (:shape :hexagon) (:radius ,radius))))
          (pm-execute :navigation trajectory))))))

(defun goto-location (horizontal-position vertical-position)
  (let ((turtle-name "turtle1"))
    (start-ros-node turtle-name)
    (init-ros-turtle turtle-name)
    (top-level
      (with-turtle-process-modules
        (process-module-alias :navigation 'simple-navigation)
        (with-designators
            ((area :location `((:horizontal-position ,horizontal-position)
                               (:vertical-position ,vertical-position)))
             (goal :action `((:type :goal) (:goal ,area))))
          (pm-execute :navigation goal))))))
