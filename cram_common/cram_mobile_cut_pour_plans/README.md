## Holding

Holding behaves like picking-up, but does no lift motion after the object was grasped. Actually, inside of CRAM, after the action of type holding was resolved the pick-up method in cram-mobile-pick-place-plans is called with the key :hold being T.

Holding needs the object it should hold, with which arm it should hold this object and how he should the object specified with the keywords
:left-hold or :right-hold. The poses of these grasp have to be definied for each object type in cram-object-knowledge. The holding action is called like this:

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

Slicing needs the object to slice since its bounding box is used to calculate how often it should be sliced. Moreover, it needs the arm which holds the arm and the start slice-position, currently specified with the keywords :right-top or :left-top. right-top indicates the
object should be sliced from the top from the right to the left. Vice-versa for left-top. This action needs for every object it should slice concerete poses which specifiy where the slicing process starts and ends. This slicing will then be repeated n times. The holding action is called like this:

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

Pouring needs the object to pour into, the arms which should be used for this action and from which side the object should be poured into.
The pose from where the tilting motion should be initiated depends on the object in which should be poured into. Therefore, one
has to specifiy its approach poses for specific grasps. In the following the object ?object-cup will be filled by something which
the robots holds in the left arm (e. g. a bottle). If the robots holds something with two or more arms (e. g. a pot) these can be passed too.

The grasping parameters depend on the robot position being infront of the object which should be filled. Otherwise, the pouring motion might
differ. For pouring from the left one should like in the example pass left-side; for pouring from the right, right-side; for pouring from the back, back; and for pouring from the front, front.

```

(desig:an action
                        (type pouring)
                        (object ?object-cup)
                        (arms (left))
                        (grasp left-side))

```

More examples of this action can be found in the demo.lisp of cram_pr2_cut_pour_demo and cram_pr2_popcorn_demo.
