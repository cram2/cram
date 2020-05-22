## Holding

Holding behaves like picking-up, but does no lift motion after the object was grasped. Actually, inside of CRAM, after the action of type holding was resolved the pick-up method in cram-mobile-pick-place-plans is called with the key :hold being T.

Holding needs the object it should hold, which arm it should use and how it should orientate its gripper to hold the object specified with the keywords :left-hold or :right-hold. The poses of these grasps have to be definied in the CRAM package cram-object-knowledge for each object type. The holding action is called like this:


```

(exe:perform (desig:an action
                               (type holding)
                               (object ?object-to-slice)
                               (arm ?arm-to-hold) 
                               (grasp ?grasp-to-hold)))


```


More examples of this action can be found in the demo.lisp of cram_pr2_cut_pour_demo.

## Slicing

Slicing is implemented as a chain of approaching motions, which are calculated during execution in cram-manipulation-interfaces::get-action-trajectory methods. 

Slicing needs the object to slice since its bounding box is used to calculate how often it should be sliced. Moreover, it needs the arm which holds the object during the slicing process. The start slice-position is currently specified with the keywords :right-top or :left-top. right-top indicates the object should be sliced from the top right to the left (vice-versa for left-top).

This action needs for every object it should slice concrete poses in its object frame which specify where each slicing process starts and ends. These slicing poses will then be used with an offset n times to slice the object in n pieces. 

The object designator `?object-to-slice` needs to hold the objects type, name and pose for the successful calculation of the slicing trajectory. Furthermore, the slicing actions allows to pass a collision-mode too (:avoid-all, :allow-all, :allow-hand). The holding action is called like this:


```

(exe:perform (desig:an action
                               (type slicing)
                               (object ?object-to-slice)
                               (arm ?arm-slicing-with) 
                               (grasp ?slice-position)))

```

More examples of this action can be found in the demo.lisp of cram_pr2_cut_pour_demo.

## Pouring

Pouring is implemented as a chain of approach and tilting events, which in the end make it possible to move arms in sequence. The tilting rotates the object from which should be poured from always around 100 degrees.

Pouring needs the object to pour into, the arms which should be used for this action and from which side the object should be filled from, which is passed with the paramter `grasp` (`:left-side`, `:right-side`, `:front`, `:back`). The object designator `?object-to-pour-into` needs to hold the objects type, name and pose for the successful calculation of the pouring trajectory. 

First, the robot approaches the object `?object-to-pour-into` with the object it holds in its arm(s). From this pose the tilting motion is initiated. Since this pose depends on the object `?object-to-pour-into`'s size and from which side it should be filled from, one has to specify these approach poses. The translation in this approach pose is used as an offset from the object and is defined in the object frame. The orientation of the approach poses is used to define the orientation of the gripper in the robot frame.

The parameter in `grasp` depends on the robot position. Therefore, the grasp `:front` matches with the x-coordinate of the robot and results in pouring from the direction the robot base is facing to (`:left-side` matches with the y-coordinate of the robot, `:back` with the -x-coordinate, `right-side` with the -y-coordinate). Due to that, the objects orientation does not matter for pouring. To summarize, for pouring from the left one should like in the example pass `:left-side`; for pouring from the right, `:right-side`; for pouring from the back, `:back`; and for pouring from the front, `:front`. Furthermore, the slicing actions allows to pass a collision-mode too (:avoid-all, :allow-all, :allow-hand).

```

(desig:an action
                        (type pouring)
                        (object ?object-to-pour-into)
                        (arms (left))
                        (grasp left-side))

```

More examples of this action can be found in the demo.lisp of cram_pr2_cut_pour_demo and cram_pr2_popcorn_demo.
