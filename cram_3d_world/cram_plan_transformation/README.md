# cram_plan_transformation

Maintainer: Arthur Niedzwiecki (aniedz@cs.uni-bremen.de)

Provides a library and examples for the transformation of CRAM plans. The execution of any CRAM plan constructs a task tree, a hierarchical data-set containing all plans, designators, motions, locations etc. that from the executed plan. With cram-prolog predicates this task tree can be traversed and analysed. See the thesis on this at https://drive.google.com/open?id=1nxjGeYHg9PsO-lZoIX6bRRWpNbE_eTiK

All the functionality is based on the task tree, resulting from an executed CRAM plan. A task tree represents an execution trace, the tree of plans and subplans that were involved in the scenario, with all the designators used within a plan. Every node in the tree stands for one plan in the complete scenario, where the node's children stand for its subplans. 

The tree and its nodes are lisp datastructures, and the task-tree-node has a slot called `code-replacement`, which is `nil` by default. But if this slot contains anything else than the initial `nil`, this code will be executed in place of the original code of the node's corresponding plan. 

So the procedure goes as follows: 

1. execute a plan
2. find patterns, nodes, designators
3. inject code into the `code-replacement` slot of specific nodes
4. execute plan again

By injecting code into specific nodes of the task tree it is possible to change the behaviour of the original plan and transform it, without changing the plan's code itself. Point 2, the search fro patterns, is done by cram-prolog predicates, and 3, the injection of code-replacements is implemented in lisp functions. 

## Tests

To execute the tests for this package, load the `cram-plan-transformation-tests` system from cram_plan_transformation. Afterwards, choose your robot description and load it, or comment-in the corresponding dependency in the `cram-plan-transformation-tests.asd` system file. Don't forget to run the `sandbox.launch` from either the pick-place-demo (pr2) or the assembly-demo (boxy). After loading the test package and robot description you can now execute this:

```commonlisp
(lisp-unit:run-tests :all :plt-tests)
```

If you don't want the debug window to pop up, go into 

```bash
roscd cram_plan_transformation/tests/
vim transformation-tests.lisp
```

and set the `*spawn-debug-window*` parameter to `nil`.

## Usage

#### Plan Transformation framework

There are a couple of function that are quite useful for managing transformations.

`apply-rules` applies one transformation rule on the task tree. Depending on the `*rule-priority*` and the `*disabled-rules*` certain rules are preferred over others. Executing `apply-rules` will execute the cram-prolog predicate associated to a transformation rule, and if the predicate yields a successful match the corresponding transformation rule is executed (using the response of the predicate).

`register-transformation-rule` is a macro that registers a transformation in the framework. It takes two arguments: the name of the transformation-function and the corresponding predicate.

```commonlisp
(register-transformation-rule both-hands-rule
                              '(task-transporting-siblings ?first-transport 
                              					 		   ?second-transport))
```

`disable-transformation-rule` excludes transformations that have been registered from `apply-rules`.

`enable-transformation-rule` does the opposite.

`prioritize-rule` takes two arguments: the name of the superior rule, and the inferior one. The predicate of a superior rule is always evaluated before an inferior rule when executing `apply-rules`.

#### Creating the task tree

To create a task tree from a plan, the tracing mechanism must be enabled. This is an example of creating a task tree for the pick and place demo:

```commonlisp
(urdf-proj:with-projected-robot
      (cet:enable-fluent-tracing)
      (demo::demo-random nil '(:bowl :cup))
      (cet:disable-fluent-tracing))
```

After execution the tracing must be disabled to prevent accidental overwriting of the task tree.

With the task tree it is now possible to analyse the executed demo. The task tree object can be found in `cpl-impl::*top-level-task-trees*`, a hash table, where `:top-level` is the key to the task tree. Executing a plan within the `urdf-proj:with-projected-robot` macro always sets the name of the task tree to `:top-level`.

#### Analysis with Predicates

All the most important predicates for analysing the task tree are already defined in the `:coe` and `:cpoe` packages, some additional predicates for pattern search and utilities are defined in `./src/predicates.lisp`. Predicates resolve into meaningful data when all their components can be evaluated positively, e.g. when there are two transporting actions in the task tree and the predicate searches for three, it fails, but the predicate that searches for two transports will be successful and can respond with data that can be used for plan transformations. 

To find a transporting task and the corresponding designator, this predicate can be used:

```lisp
(prolog `(and (top-level-name ?tl-name)
              (top-level-path ?tl-path)
              (cpoe:task-transporting-action ?tl-name ?tl-path ?task ?designator)))
```

To find two transporting actions, a bigger predicate is needed. The predicate will be build up in a few steps, beginning with the header. This predicate is necessary for modifying the tasks tree in a way, such that the robot will use both of the grippers for transporting two objects from the same place; also called `both-hands-rule`.

```
(<- (task-transporting-siblings (?first-path ?first-fetching-desig)
                                (?second-path ?first-delivering-desig))
```

The predicate does not need any bound variables, meaning, it can be called with four arbitrary variable names. After the predicate was successful, we will have 4 return values, holding the data we need for transforming the plan. This example works with the data from the demo-random with :bowl and :cup.

* `?first-path`: address to the bowl **transport** 

* `?first-fetching-desig`: designator of the bowl *fetch* 

* `?second-path`: address to the cup *delivery*

* `?first-delivering-desig`: designator of the bowl *delivery*

It can be called like this: 

`(prolog '(task-transporting-siblings ?part-a ?part-b))` or like this 

`(prolog '(task-transporting-siblings (?part-a-1 ?part-a-2) (?part-b-1 ?part-b-2)))`

Later, when the data is applied on the task tree, the following will happen: The addresses are pointing to the nodes in the task tree, that are going to be manipulated. The bowl **transport** node will be changed to only contain the bowl *fetch*, and the cup *delivery* will be extended to also contain the bowl *delivery*. This way, the order of *fetch, deliver, fetch, deliver* is changed to *fetch, fetch, deliver, deliver*.

From the header ongoing the predicate will find two transporting actions.

```commonlisp
(<- (task-transporting-siblings (?first-path ?first-fetching-desig)
                                (?second-path ?first-delivering-desig))
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (cpoe:task-transporting-action ?top-level-name ?path ?second-task ?_)
    (cpoe:task-transporting-action ?top-level-name ?path ?first-task ?_)
```

For security purposes it is not allowed to transform any node that has been involved in a transformation already. Transformed nodes contain a piece of code in their `code-replacement` slot, that will be executed in place of their original code reference. Checking for this slot will reveal and exclude nodes, that are already 'tainted'.

```commonlisp
(<- (task-transporting-siblings (?first-path ?first-fetching-desig)
                                (?second-path ?first-delivering-desig))
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (cpoe:task-transporting-action ?top-level-name ?path ?second-task ?_)
    (cpoe:task-transporting-action ?top-level-name ?path ?first-task ?_)
    (without-replacement ?first-task)
    (without-replacement ?second-task)
```

To further specify the validity of the transporting action, the tasks must not be the same, but the origin of the transports are approximately equal.

```
(<- (task-transporting-siblings (?first-path ?first-fetching-desig)
                                (?second-path ?first-delivering-desig))
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (cpoe:task-transporting-action ?top-level-name ?path ?second-task ?_)
    (cpoe:task-transporting-action ?top-level-name ?path ?first-task ?_)
    (without-replacement ?first-task)
    (without-replacement ?second-task)
    (not (== ?first-task ?second-task))
    (task-location-description-equal ?first-task ?second-task)
```

The rest of the predicates retrieves the addresses and designators:

```
(<- (task-transporting-siblings (?first-path ?first-fetching-desig)
                                (?second-path ?first-delivering-desig))
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (cpoe:task-transporting-action ?top-level-name ?path ?second-task ?_)
    (cpoe:task-transporting-action ?top-level-name ?path ?first-task ?_)
    (without-replacement ?first-task)
    (without-replacement ?second-task)
    (not (== ?first-task ?second-task))
    (task-location-description-equal ?first-task ?second-task)
  ;; Bind the ?first-path variable to the bowl transport address
    (coe:task-full-path ?first-task ?first-path)
  ;; Extract the path of the cup transport
    (coe:task-full-path ?second-task ?second-transporting-path)
  ;; Bind the ?first-fetching-desig to the bowl fetch
    (cpoe:task-fetching-action ?top-level-name ?first-path ?_ ?first-fetching-desig)
  ;; Bind the ?first-delivering-desig to the bowl delivery
    (cpoe:task-delivering-action ?top-level-name ?first-path ?_ ?first-delivering-desig)
  ;; Extract the cup delivery task
    (cpoe:task-delivering-action ?top-level-name ?second-transporting-path ?second-delivering-task ?_)
  ;; Bind the ?second-path to the cup delivery address
    (coe:task-full-path ?second-delivering-task ?second-path))
```

#### Transformation Functions

In `./src/demo-transformation-rules.lisp` there are a few transformation functions. The `both-hands-rule` has already been mentioned in the analysis section, and this is how the data from the predicate is used on the task tree.

The result of a predicate is a lazy list, consisting of key-value bindings. Those bindings are passed to the transformation functions as input parameters. The transformation function uses the data in the bindings (node addresses and designators) to inject code-replacement into nodes in the task tree. Every transformation function contains this piece of code:

```commonlisp
(cpl-impl:replace-task-code '(NAME-OF-THE-TRANSFORMATION)
                                #'(lambda (&rest desig)
                                	;; ignore the original designator if you want
                                    (declare (ignore desig))
                                    ;; or execute it
                                    (exe:perform (car desig))
                                    ;; here comes
                                    (your-injected-code))
                                path-to-the-node-to-inject-the-code-in
                                (cpl-impl::get-top-level-task-tree top-level-name))
```

This function call injects a function of signature `(lambda (&rest desig) do-something)` into the code-replacement slot of the node at the `path-to-the-node-to-inject-the-code-in`. The `NAME-OF-THE-TRANSFORMATION` can be chosen by the developer, it will be used as part of the new address of the manipulated node. The last parameter of the function is the manipulated task tree.

Following is the transformation function for the `both-hands-rule`:
```commonlisp
(defun both-hands-rule (lazy-bindings)
  (destructuring-bind
      ((key transport-path 1st-fetch-action)
       (other-key 2nd-deliver-path 1st-deliver-action))
      (cut:lazy-car lazy-bindings)
    (declare (ignore key other-key))
    (cpl-impl:replace-task-code '(BOTH-HANDS-TRANSFORM-1)
                                 #'(lambda (&rest desig)
                                     (declare (ignore desig))
                                     (exe:perform 1st-fetch-action))
                                 transport-path
                                 (cpl-impl::get-top-level-task-tree top-level-name))
    (cpl-impl:replace-task-code '(BOTH-HANDS-TRANSFORM-2)
                                 #'(lambda (&rest desig)
                                     (exe:perform 1st-deliver-action)
                                     (exe:perform (car desig)))
                                 2nd-deliver-path
                                 (cpl-impl::get-top-level-task-tree top-level-name))))
```

