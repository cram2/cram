(in-package :tut)

(cram-process-modules:def-process-module turtle-actuators (action-designator)
  (roslisp:ros-info (turtle-process-modules)
                    "Turtle navigation invoked with action designator `~a'."
                    action-designator)
  (destructuring-bind (cmd action-goal) (reference action-designator)
    (ecase cmd
      (shape
         (call-shape-action
          :edges (turtle-shape-edges action-goal)
          :radius (turtle-shape-radius action-goal))))))


(defmacro with-turtle-process-modules (&body body)
  `(cpm:with-process-modules-running
       (turtle-actuators)
     ,@body))

(defun draw-hexagon (r)
  (let ((turtle-name "turtle1"))
    (start-ros-node turtle-name)
    (init-ros-turtle turtle-name)
    (top-level
      (with-turtle-process-modules
        (cpm:process-module-alias :manipulation 'turtle-actuators)
          (cram-language-designator-support:with-designators
            ((trajectory (action `((type shape) (shape hexagon) (radius ,r)))))
            (cpm:pm-execute :manipulation trajectory))))))

(cram-process-modules:def-process-module turtle-navigation (location-designator)
  (roslisp:ros-info (turtle-process-modules)
                    "Turtle navigation invoked with location designator `~a'."
                    location-designator)
  (let ((target-pose (reference location-designator)))
       (print (cl-transforms:origin target-pose))
       (move-to (cl-transforms:origin target-pose))))

(defun goto-location (location-desig)
  (let ((turtle-name "turtle1"))
    (start-ros-node turtle-name)
    (init-ros-turtle turtle-name)
    (top-level
      (cpm:with-process-modules-running (turtle-navigation)
        (cpm:process-module-alias :navigation 'turtle-navigation)
            (cpm:pm-execute :navigation location-desig)))))

