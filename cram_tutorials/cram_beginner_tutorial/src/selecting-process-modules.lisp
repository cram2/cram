(in-package :tut)

(def-fact-group navigation-process-modules (available-process-module
                                            matching-process-module)
  (<- (available-process-module actionlib-navigation))
  (<- (available-process-module simple-navigation))

  (<- (matching-process-module ?designator actionlib-navigation)
    (desig-prop ?designator (:type :shape)))
  (<- (matching-process-module ?designator simple-navigation)
    (desig-prop ?designator (:type :goal))))

(defun perform-some-action (action-desig)
  (let ((turtle-name "turtle1"))
    (start-ros-node turtle-name)
    (init-ros-turtle turtle-name)
    (top-level
      (with-turtle-process-modules
        (pm-execute-matching action-desig)))))
