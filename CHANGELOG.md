
ChangeLog
=========

### [0.3.0] - 2017-03-15

2016-11-23 [pr2-ll] when initializing service also return the service object

2016-11-23 [cram_pr2_arm_kinematics] contains ik / fk service clients: Supports moveit and minimal moveit from pr2_arm_kinematics services.

2016-11-24 [cram_pr2_arm_kinematics] finalized cram_pr2_arm_kinematics: it now implements the compute-ik function from cram_robot_interfaces

2016-11-24 [cram_pr2_arm_kinematics] exported moveit-or-arm-kinematics parameter

2016-11-24 [pr2_descr] exported PR2 symbol

2016-11-24 [pr2_descr] restructured and cleaned up

2016-11-24 [pr2_descr] got rid of side->ik-group function: use the corresponding prolog predicate instead

2016-11-24 [projection_pms] adopted the slightly different new compute-ik

2016-12-02 [pr2_launch_files] some tweaks


### [0.2.2] - 2016-11-15

2016-02-22 [cram_pr2_desc] there's a new ROBOT-ODOM-FRAME predicate

2016-02-22 CRAM_TRANSFORMS_STAMPED -> CRAM_TF

2016-02-24 [reachability_cm] some fixes for the costmap, still not working 100%

2016-02-24 [pr2_low_level] added low-level functions for talking directly to PR2's controllers.

2016-03-07 [pr2_pms] added initial package for pr2's new pms

2016-03-09 moved very specific things from pr2_desc to pr2_desigs

2016-03-09 [pr2_desc] doesn't depend on btr anymore

2016-05-03 [pr2_desc] added gripper forces for objects

2016-06-30 [pr2_synch_proj_pms] fix error when carrying objects with both hands

2016-06-30 [pr2_ll] giskard interface

2016-06-30 [pr2-pms] process module for giskard: actions look like this: (desig:an action (to move) (right arm) (to ((0.5 0 1) (0 0 0 1))))

2016-07-01 [pr2_desigs] added dependency on bullet reasoning

2016-07-07 [reachability_cm] Reachability map now supports Moveit IK service calls.

2016-07-14 [pr2-pms] action designators for grasping with both arms

2016-07-19 [nav_pm] Added `:timeout` as optional designator field for navigation actions: the `:timeout` parameter directly affects the `actionlib-lisp` timeout when calling `send-goal-and-wait`. The default value is 10.0 seconds.

2016-07-19 [nav_pm] Slight change in designator interface for navigation pm to make timeouts work: since the `action-desig` designator rule (which is used to invoke `pm-run` when activating process modules) doesn't allow more than one parameter (and that one need to be a designator), the timeout is now resolved within the lisp functions rather than the prolog rules.

2016-07-20 Don't prepend "/" when defining tf frames

2016-07-21 [reachability_cm] Sped up reach.map generation, and made it applicable to other robots besides the pr2.

2016-07-25 [pr2-ll] added first failure definitions

2016-07-25 [pr2-pms] added more specifics to designators of PMs: now performing action designators works on both PMs and plans, so we need to make sure the PMs say exactly which designators they match to, to avoid ambiguity

2016-07-26 [pr2-ll] added VALUES-CONVERGED functions

2016-07-26 [pr2-pms] more precise matching desingators for gripping actions

2016-07-26 [pr2-pms] added another example of a gripping action

2016-07-27 [pr2-ll] added goal convergence checks

2016-07-27 [cram-pr2-desc] implemented ROBOT-TOOL-FRAME predicate

2016-07-27 [pr2-pms] updated pms to work with new pr2-ll apis

2016-07-27 [pr2-ll] moved launch files to pr2-plans

2016-07-27 [pr2-plans] added a package to put giskard plans into

2016-07-27 [pr2-launch-files] added package to store pr2 launch files

2016-07-27 [pr2-ll] exported useful symbols

2016-07-27 [pr2-ll] added initial json interface to robosherlock

2016-07-27 [pr2-pms] ptu action is exported from pr2-ll now

2016-07-27 [cram-pr2-desc] implemented ROBOT-TOOL-FRAME predicate

2016-07-22 [ptu-pm] Made sure than head-movement eventually gets its poses transformed: head-movement should never fail; that's why tf transformations are trying indefinitely.

2016-07-28 [manip_pm] Incorporate an optionally external prolog rule for setting the `close-radius` property for grasping

2016-07-28 [manip_pm] Another instance of `make-assignment` that required `close-radius`

2016-07-29 Setting `:timestamp` to `0.0` for `on-event` `robot-state-changed`: the timestamp setting in the robot-state-changed decides which timestamp is sent to tf for `lookup-transform`. It defaults to the current time, which can result in tf running into its timeout when waiting for transforms. Manually setting it to `0.0` results in tf using the latest transform available (which usually works). If we ever want to use tf's time travel feature, this becomes problematic. But for the time being, this is working perfectly.

2016-07-29 [pr2-ll] proper robosherlock interface

2016-07-29 [pr2-pms] proper implementation of perception process module

2016-07-29 [pr2-plans] added dependency on cram plan lib

2016-08-01 [pr2-pms] there's some bug in pattern matching that disappears sometimes...

2016-08-02 [manip-pm] Make sure that transforms use the latest transform available

2016-08-02 [manip-pm] How an object is grasped can also be noted in the object designator

2016-08-04 [reachability-cm] now store the moveit-generated map in the appropriate object.

2016-07-05 [reachability-cm] now calling normal services by default.

2016-07-07 [reachability-cm] Generating a reachability map based on Moveit IK service calls.

2016-07-21 [reachability-cm] Sped up reach.map generation, and made it applicable to other robots besides the pr2.

2016-08-05 [pr2-ll] always send goals to both arms in giskard

2016-08-05 [pr2-plans] worked on grasping

2016-08-05 [pr2-ll] proper json parser

2016-08-08 [pr2-ll] some updates for gripper

2016-08-08 [pr2-plans] minor plan adjustment to grasp bottles

2016-08-08 [pr2-ll] use *tf-default-timeout* always

2016-08-10 [pr2-ll] robosherlock interface supports duplicate keys now: had to use a-lists instead of hash tables

2016-08-10 [pr2-ll] visualize-marker accepts a frame name now

2016-08-10 [pr2-ll] json parsing add flattening of nested key-values

2016-08-10 [pr2-plans] unified grasping somewhat. Next step is putting grasp poses into designators

2016-08-10 [pr2-plans] unified pregrasping somewhat.

2016-08-11 [pr2-description] Added predicates to get arm links and joints and hand-links

2016-08-15 [manipulation-pm] Code refactoring to make use of motion managers.

2016-08-16 [pr2-ll] visualize-marker can also visualize lists of poses

2016-08-16 [pr2-pms] better designator interface for moving arm

2016-08-17 [pr2-ll] in visualizing allow empty frame arguments

2016-08-17 [pr2-ll] gripper action accepts a list of sides now

2016-08-17 [pr2-pms] arm pm accepts now a list of poses for one arm and single pose for other

2016-08-18 [manipulation-pm] Added make-fallback-converter.

2016-08-18 [manipualtion-pm] Putting down safety distance from 0.1m to 0.01m

2016-08-18 [pr2-ll] visualize-marker don't crash on null pointers

2016-08-19 [manipualtion-pm] Fixed designator resolution for park action-handler.

2016-08-19 [manipualtion-pm] Ignore the object when not set

2016-08-19 [manipualtion-pm] Do preliminary IK checks before doing motion planning

2016-08-19 [manipualtion-pm] Default is giskard, not moveit

2016-08-19 [pr2-ll] tiny cleanup of RS interface

2016-08-19 [pr2-ll] in grippers perform two-gripper actions in parallel

2016-08-19 [pr2-pms] better gripper interface + renamed action types: calling action designators that go straight to PMs motions (will change to motion desig later), so added suffix -motion to type names

2016-08-19 [pr2-plans] generalized pick and place plans

2016-08-19 [pr2-plans] cartesian pouring plans

2016-08-22 [manipualtion-pm] Allow blind, aka no prelim IK solution checking, `assume`

2016-08-22 [manipualtion-pm] Properly handle cost IK checks

2016-08-22 [manipualtion-pm] First detach, then unhand

2016-08-22 [manipualtion-pm] Motion manager goal specification

2016-08-25 [pr2-ll] added new function GET-ARM-JOINT-STATES

2016-08-25 [pr2-ll] giskard joint interface

2016-08-25 [pr2-plans] got rid of tf testing code: cl-tf is awesome now.

2016-08-25 [pr2-description] Predicates to read joint properties from the urdf, and to describe groups of links/joints.

2016-08-26 [pr2-description] Quick work-around for a problem in the real PR2 urdf.

2016-08-26 [pr2-desc] commented out register-ros-init-func: it's too risky

2016-08-26 [pr2-ll] added giskard yaml interface

2016-08-26 [pr2-ll] added Mihais yaml splicer

2016-08-26 [pr2-plans] better grasping

2016-08-26 [pr2-plans] read giskard yaml files and execute them

2016-08-27 [pr2-plans] fixed little bug in visualization

2016-08-27 [pr2-ll] object labels are called CLASS now, not type

2016-08-27 [pr2-pms] designator of type move-joints-motion: for sending joint commands

2016-08-27 [pr2-ll] increased giskard timeout to 20: sometimes front grasps actually take that long...

2016-08-27 [pr2-ll] in robosherlock substitute calls to type cup to cad-model

2016-08-27 [pr2-plans] joint poses to get arms out of sight

2016-08-27 [pr2-plans] placing at object desig pose and better pouring

2016-08-29 [pr2-plans] when placing object designator preceeds AT property

2016-08-29 [pr2-plans] for pouring added a placeholder for incorporating learned constraints

2016-08-29 [pr2-ll] better merging of designator descriptions for robosherlock

2016-08-30 [pr2-plans] service client for learned constraints

2016-08-30 [pr2-descr] only use robot urdf if it's on the param server

2016-08-30 [pr2-plans] pouring with giskard

2016-08-30 [pr2-plans] added example input designator

2016-08-30 [pr2-ll] in ptu actions use point frame if given

2016-08-30 [pr2-ll] in robosherlock don't override type cup with cad-model: instead use both annotations

2016-08-30 [pr2-plans] separated pouring designator resolutions for yaml and cartesian

2016-08-30 [pr2-plans] more grasping

2016-08-30 [pr2-plans] some logging for pick and place

2016-08-30 [pr2-plans] tilting phase is now called tilt-down

2016-08-30 [pr2-plans] better pouring plans

2016-08-31 [pr2-plans] refactoring - renaming

2016-08-31 [pr2-plans] object-in-hand functionality

2016-08-31 [pr2-plans] put down object-in-hand

2016-08-31 [pr2-plans] when pick-and-placing cups use cad model fitter

2016-08-31 perceiving multiple objects of the same description

2016-08-31 [pr2-plans] don't be so upset when object is already in hand

2016-08-31 [pr2-plans] better pouring

2016-09-01 nice and robust pouring plan

2016-09-01 [pr2-plans] start the demo from a particular step

2016-09-01 [pr2-plans] fixed pouring initial joint config which didn't really help

2016-09-03 [pr2-plans] Implemented aux function to complete yaml-templates with constraints for pouring with giskard.

2016-09-03 [pr2-plans] incorporated Georg's PR

2016-09-03 [pr2-plans] fixed coordinates for Yuen's model

2016-09-03 [pr2-plans] added pour volume and liquid volume for Yuen's model

2016-09-03 [pr2-plans] use tested liquid volumes

2016-09-03 [pr2-ll] rename ID of object into CLUSTER-ID

2016-09-03 [pr2_plans] pouring with Yuen's model

2016-09-03 [pr2-plans] only pour into second cup if its in the air

2016-09-04 [pr2-plans] stand in front of object when picking up per default

2016-09-04 [pr2-plans] use roslisp:wait-duration instead of cpl:sleep: the later might have some problems

2016-08-29 [ptu-pm] Make sure pose frame is correct

2016-08-29 [manipulation-pm] Object being held too high

2016-08-31 [manipulation-pm] When assuming grasp pose, act `blindly`. Also, when `ignore-collisions` is active, don't do preliminary IK checks

2016-08-31 [manipulation-pm] (Pre-)grasp offset fixes

2016-08-31 [manipulation-pm] When checking for reachability, ignore collisions when checking for the `grasp` pose

2016-08-31 [manipulation-pm] Allow ignoring collisions for `grasp` when checking (pre-)grasp pose reachability

2016-08-31 [manipulation-pm] Make sure the right message type is serialized when visualizing object handles

2016-08-31 [manipulation-pm] Annotate `lowering-object` activity for openEASE logging

2016-09-01 [manipulation-pm] Allow specifying arms when asking for gripper offsets, and include them in grasp reachability calculation

2016-09-01 [manipulation-pm] Partial fix for reachability cost calculation

2016-09-01 [manipulation-pm] Parking: Ignore collisions

2016-09-01 [manipulation-pm] Logging fix

2016-09-01 [manipulation-pm] `relative-pose` calculation by type

2016-09-03 [manipulation-pm] Dummy park handler

2016-09-03 [manipulation-pm] Default park poses

2016-09-03 [manipulation-pm] Parking handler

2016-11-08 [pr2] plans grasp cutlery with a stronger force

2016-11-08 [pr2-plans] temporary ugly fix for logging to log complex actions

2016-11-10 [pr2-reachability] don't :use cl-transforms-stamped

2016-11-11 [pr2-plans] plugged in costmaps for reachability reasoning

2016-11-15 cleaned up package.xmls


### [0.2.1] - 2016-01-28
