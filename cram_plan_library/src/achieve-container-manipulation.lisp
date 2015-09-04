(in-package :plan-lib)

(def-goal (achieve (container-opened ?obj ?angle))
  (ros-info (achieve plan-lib) "(achieve (container-opened))")
  (with-designators
      ((open-loc :location `((:to :open) (:to :reach)
                             (:obj ,?obj)))
       (open-action :action `((:type :trajectory)
                              (:to :open) (:obj ,?obj)
                              (:angle ,?angle))))
    (at-location (open-loc)
      (perform open-action)))
  ?obj)
