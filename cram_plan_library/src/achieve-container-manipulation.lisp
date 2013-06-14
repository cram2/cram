(in-package :plan-lib)

(def-goal (achieve (container-opened ?obj ?angle))
  (ros-info (achieve plan-lib) "(achieve (container-opened))")
  (with-designators
      ((open-action (action `((type trajectory)
                              (to open) (obj ,?obj) (angle ,?angle)))))
    (perform open-action))
  ?obj)
