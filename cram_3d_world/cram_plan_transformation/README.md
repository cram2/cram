# cram_plan_transformation

Maintainer: Arthur Niedzwiecki (aniedz@cs.uni-bremen.de)

Provides a framework and examples for the transformation of CRAM plans. See the full thesis on this at https://drive.google.com/open?id=1nxjGeYHg9PsO-lZoIX6bRRWpNbE_eTiK

Plan transformation is based on CRAM's task tree. When a CRAM plan is run, each action designator generates a node in the task tree, starting from the root (top-level). If an action designator consists of other actions (which they usually do), those actions generate nodes as well and appear as child-nodes in the node they were called in. This hierarchy of actions within actions within actions spans a task tree, whose leafs end in atomic motions. It is called 'task tree', because every node contains a task, which holds a lot of information about the corresponding designator.

A task tree is automatically generated whenever a plan is run. If a different plan is run, the current task tree is overwritten with the new plan's structure. But if the same plan is run repeatedly with the same structure of actions within actions, the task tree structure also stays the same. While repeated executions of a plan only update the designator results in their corresponding nodes, it is also possible to change the course of actions without touching their original implementation.

Each node has a unique path, from the root down along its children of children, leading to a specific node and its task. A task is a lisp object and contains a field named `code-replacement`, which is `nil` by default. When it comes to executing the action corresponding to that task, and the `code-replacement` field is not `nil` but filled with a completely different action instead, this code replacement is executed **instead** of the original action.

**Example 1**: With the path to a task, whose action should be ignored in the next plan execution, its `code-replacement` can be addressed to a function that does nothing. The field `code-replacement` must contain a function of the form `(function (&rest desig) &body)` while the input `desig` is a list, whose first value is the originally executed action of the corresponding task. A function that does nothing then looks like this:

```commonlisp
(defun ignore-desig (&rest desig)
   (declare (ignore desig))
   T)
```

To remove the execution of an obsolete action, which is contained in a task whose path is known (`obsolete-task-path`), the following function sets the `code-replacement` to `ignore-desig`:

```commonlisp
(cpl-impl:replace-task-code
   "OBSOLETE-TASK-1" ;; the task name, never used in any context
   #'ignore-desig ;; the function to replace the original code with
   obsolete-task-path ;; the path to the affected node
   :top-level) ;; the name of the task tree, default is :top-level
```

**Example 2**: If a couple of action designators (`actions-list`) need to be executed instead of an other action, their execution can be injected into a task with the path `modified-task-path` like this:

```commonlisp
(cpl-impl:replace-task-code
   "MODIFIED-TASK-1"
   #'(lambda (&rest desig)
        (declare (ignore desig))
        (mapcar #'exe:perform actions-list))
   modified-task-path 
   :top-level)
```

## Usage

### Example and Explanation

The two examples above - deleting and injecting code - are the bread and butter for transforming a task tree. Each transformation consists of a code-injection and a code-deletion part. Among the currently implemented predicates there is one transformation rule, that lets the robot transport two objects to the same location simultaneously, instead of one by one. This is done by postponing the delivery of the first object.

|               | Original Plan                         | Transformed Plan                                       |
| ------------- | ------------------------------------- | ------------------------------------------------------ |
| 1st Transport | 1st search & access goal location     | 1st search & access goal location                      |
|               | 1st search & access fetching location | 1st search & access fetching location                  |
|               | 1st search & fetch object             | 1st search & fetch object                              |
|               | 1st deliver object                    | **(ignore (1st deliver object))**                      |
|               | 1st seal fetching location            | 1st seal fetching location                             |
|               | 1st seal goal location                | 1st seal goal location                                 |
| 2nd Transport | 2nd search & access goal location     | 2nd search & access goal location                      |
|               | 2nd search & access fetching location | 2nd search & access fetching location                  |
|               | 2nd search & fetch object             | 2nd search & fetch object                              |
|               | 2nd deliver object                    | **(inject (1st deliver object) (2nd deliver object))** |
|               | 2nd seal fetching location            | 2nd seal fetching location                             |
|               | 2nd seal goal location                | 2nd seal goal location                                 |

After this transformation the robot fetches the first object and holds it there while using the other gripper for all following tasks, until the first object can be delivered at the target location. For this specific transformation, the following parameters need to be acquired:

* path to 1st deliver object - ignore the designator
* path to 2nd deliver object - inject actions here
  * designator:  1st deliver object  - inject this action here
  * designator: 2nd deliver object - inject this action here

A good question would be, why the *2nd deliver object* action designator is needed when the code is injected into its exact path anyway. The answer is, it's more generic. Any transformation-parameter can be generalized into two lists: 

1. a list of designator-path pairs, where each element is a list of actions and a path where the actions are injected into
2. a list of paths to ignore

For example, this is the first part of the transformation-parameter for the transformation above:

```
(  
   ((1st-deliver-designator
     2nd-deliver-designator)
    2nd-deliver-path)
)
```

It contains only one element: the injection of both delivery tasks into the second delivery node. There can be multiple such action-path pairs, each injecting a list of actions into one path, e.g. can two actions be switched in places by injecting one into another, and the other one into the first.

Since the first delivery is injected into the second delivery it would now be executed two times, first at its original time, and later at the injected time, therefore the first execution must be ignored. For this purpose the second part of the transformation-parameter contains a list of paths, whose actions should be ignored.

The final transformation-parameter for this example transformation is the concatenation of the first and second part:

```
(
   (((1st-deliver-designator
      2st-deliver-designator)
     2st-deliver-path))
   (1nd-deliver-path)
)
```

More generic, the transformation-parameter for any transformation has the following structure:

```
(
   (((designator-a
      designator-b
      designator-c)
     injected-path-1)
    ((designator-x
      designator-y)
     injected-path-2)
    ...)
   (ignored-path-k 
    ignored-path-l
    ignored-path-m
    ...)
)
```

### Plan Transformation Framework

In `framework.lisp` a function is defined, that takes the transformation-parameter as input and applies the code replacements onto the task tree. The function loops through the first part of the transformation-parameter, injects the actions into their respective target nodes, and ignores the tasks defined in the second part of the transformation-parameter. 

The most difficult part is finding the right values for the transformation-parameter. This search is done with predicates written in cram-prolog. Each such predicate can be viewed as a transformation rule, because it first searches the task tree for a suitable, transformable pattern, then extracts the needed actions and paths and provides them in the structure of a transformation-parameter as explained above.

To get an idea on how to construct such predicates, the transformation rule in the example is already implemented in `predicates.lisp` and is called `task-transporting-both-hands-to-target`. Like every transformation-rule predicate, both parameters of this predicate are passed unbound and will be filled  with the transformation-parameter. A transformation-rule predicate only passes, if the rule is applicable onto the current task tree, and evaluates to NIL if it is not applicable.

A transformation-rule can then be registered  as possible transformation with the `register-transformation-rule` macro:

```
(register-transformation-rule
 both-hands-to-target '(task-transporting-both-hands-to-target ?transform-data ?obsolete-paths))
```

Registering a transformation rule enables the framework to see and use the constructed rule. With the function `apply-rules` all registered transformation rules are run through the cram-prolog interpreter, and if they are applicable, are applied onto the task tree. Each execution of `apply-rules` only applies one transformation rule at a time. If called multiple times, the registered transformation rules are checked for applicability and applied again, if any applicable rule is found. 

#### Constructing Transformation Rule Predicates

It is imperative to write transformation rules that terminate. Cram-prolog has not the healthiest way of allocating memory, so the predicates must be constructed thoughtfully and precise. Even if it seems superfluous, constraining predicates can help the performance a lot. As an example of seemingly superfluous code, the first lines of the predicate `task-transporting-from-similar-origin`  are shown:

```commonlisp
;; improvable example
(<- (task-transporting-from-similar-origin ?first-task ?second-task
                                           ?fetch-task-1 ?fetch-task-2
                                           ?origin-location)
    (bound ?first-task)
    (bound ?second-task)
    (top-level-name ?top-level-name)
    (coe:task-full-path ?first-task-1 ?first-path)
    (cpoe:task-fetching-action ?top-level-name
                               ?first-path   ;; path of the parent
                               ?fetch-task-1 ;; unspecfied task
                               ?fetch-desig-1)
    ...)

;; current implementation
(<- (task-transporting-from-similar-origin ?first-task ?second-task
                                           ?fetch-task-1 ?fetch-task-2
                                           ?origin-location)
    (bound ?first-task)
    (bound ?second-task)
    (top-level-name ?top-level-name)
    (coe:subtask ?first-task ?fetch-task-1) ;; specifying as subtask
    (coe:task-full-path ?fetch-task-1 ?fetch-path-1)
    (cpoe:task-fetching-action ?top-level-name
                               ?fetch-path-1 ;; path of the searched task
                               ?fetch-task-1 ;; specified task
                               ?fetch-desig-1)
    ...)
```

The predicate `bound` tells, that `?first-task` and `?second-task` need to be provided with a value, `top-level-name` gives the name of the task tree, `coe:task-full-path` returns the path to a given task, `coe:subtask` returns each subtask of a given root-task, the predicate `cpoe:task-fetching-action` searches depth-first for an action of type :fetching, in all nodes and child-nodes of the given path. 

In the upper, improvable example, the fetching task is found by using the path of its parent `?first-path`. The search for the fetching action starts from the  `?first-task`, checks if the `?first-task` is a fetching action, then goes down its child-nodes and continues searching depth-first. Since transporting actions have a lot going on besides fetching and delivering and the predicates go depth-first, the search takes quite a while to finally reach the fetching action.

Instead, when specifically declaring the searched `?fetch-task-1` to be a direct subtask of `?first-task` through the predicate `coe:subtask`, the search focuses on direct subtasks of `?first-task`, while ignoring the depths below. Prior knowledge of the task tree's structure can help building predicates a lot.

## Tests

To test the plan transformation by hand, use any projection demo including two transporting actions to the same target. The transporting actions must be achievable with either gripper. First execute the demo until it is successful, then call `plt:apply-rules` and execute the demo again. 

An autonomous test can be found in the `cram-plan-transformation-tests` package. Simply load the package and call

```commonlisp
(lisp-unit:run-tests :all :plt-tests)
```

Currently, the implementation of `cram-plan-transformation-tests` loads and uses the household-demo in `cram_projection_demos`, transporting bowl and breakfast cereal. The projection window will not open unless the test fails. Optionally, go into `tests.lisp` and toggle `btr-belief:*spawn-debug-window*` to make the window spawn when the test is executed. Upon a failed test, check the error message and try again, demos sometimes fail non-deterministically.