cram_urdf_environment_manipulation
=================================

CRAM packages related to ...


Navigating towards a location for conveniently opening / closing drawer:

```lisp

(urdf-proj:with-projected-robot
    (let ((?arm :left)
          (?container-desig
            (an object
                (type drawer)
                (urdf-name sink-area-left-middle-drawer-main)
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

(urdf-proj:with-projected-robot
  (perform (an action
               (type opening)
               (arm right)
               (object (an object
                           (type drawer)
                           (urdf-name sink-area-left-upper-drawer-main)
                           (part-of kitchen)))
               (distance 0.3))))

```

Opening or closing a fridge:

```lisp

(urdf-proj:with-simulated-robot
  (exe:perform
   (an action
       (type opening)
       (arm right)
       (distance 0.6)
       (object
        (an object
            (type fridge)
            (urdf-name iai-fridge-door)
            (part-of kitchen))))))

```

Navigating and opening / closing the fridge (this requires fetch and place plans library):

```lisp

(urdf-proj:with-simulated-robot
  (exe:perform
   (an action
       (type accessing)
       (arm right)
       (distance 0.4)
       (location
        (a location
           (in
            (an object
                (type container-revolute)
                (urdf-name iai-fridge-door)
                (part-of kitchen))))))))

```

When using this package some assumptions are put on the URDF of the environment, which is to be manipulated:

- There are 3 types of joints: :FIXED, :PRISMATIC, :REVOLUTE
  - so when we find a joint that's not :FIXED we know it's the joint we are interested in to manipulate
- Connecting joints (those which are revolute or prismatic and connect a container with the rest of the environment)
  - Have proper limits set
  - Are above their container-link in the URDF hierarchy
    - That's why for opening the fridge one has to open the fridge-door instead of the fridge itself
- Handles
  - Handle-links have the string "handle" in their name
    - Otherwise error in find-handle-under-link
  - Prismatic containers have a horizontal handle
  - Revolute containers have a vertical handle
  - Exceptions in the iai-kitchen:
    - "oven_area_area_left_drawer_main"
    - "oven_area_area_right_drawer_main"
    - both are prismatic but have vertical handles
- Rotation axis for revolute joints is Z (in world)
- Every container-link has a rigid-body with a pose
- A container's URDF joints don't branch. (Only the first "to-joint" of an link is followed when going down the hierarchy to find a handle or joint-type)
