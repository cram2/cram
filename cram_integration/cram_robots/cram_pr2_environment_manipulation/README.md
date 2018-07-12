cram_pr2_environment_manipulation
=================================

CRAM packages related to ...


Navigating towards a location for conveniently opening / closing drawer:

```lisp

(pr2-proj:with-projected-robot
    (let ((?arm :left)
          (?container-desig
            (an object
                (type drawer)
                (urdf-name sink_area_left_middle_drawer_main)
                (part-of kitchen))))
      (exe:perform (an action
                       (type navigating)
                       (location
                        (a location
                           (reachable-for pr2)
                           (arm ?arm)
                           (object ?container-desig)))))))

```

Opening or closing a drawer:

```lisp

(pr2-proj:with-projected-robot
    (perform (an action
              (type opening)
              (arm left)
              (object (an object
                       (type drawer)
                       (urdf-name sink_area_left_upper_drawer_main)
                       (part-of kitchen))))))

```