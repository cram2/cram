
ChangeLog
=========

### [0.3.0] - 2017-03-15

2016-12-02 [interfaces] slightly different compute-ik interface

2016-12-23 [costmaps] added a default DESIG-COSTMAP definition: otherwise if there are no costmaps defined there is an annoying warning

2017-01-11 [plan-lib] after manipulating an object return the resulting object desig

2017-01-16 [commander] Initial commit

2017-01-16 [commander] working on the ROS interface

2017-01-18 [commander] encoding designators with JSON

2017-01-19 [commander] communication between commander and robots more or less done

2017-02-01 [commander] changed perform_designator from service into actionlib

2017-02-06 [commander] allow using cl-transforms:pose-s in the action designator

2017-02-08 [commander] working on the perform action

2017-02-08 [commander] implemented stopping command

2017-02-17 [commander] reimplemented communication using services not actions: action server is damn buggy :'(

2017-02-21 [costmap] added VISUALIZATION-Z slot to location costmap

2017-02-22 [costmaps] bugfix: be consistent in the usage of ROUND or TRUNCATE, don't interchange

2017-03-13 [tf] added 4 util functions from sherpa

2017-03-13 [tf] added prolog utility rule from sherpa

2017-03-13 [robot_interfaces] added 3 helper functions from sherpa

2017-03-14 moving commander from sherpa to main CRAM repos

2017-03-14 [commander] dependencies

2017-03-15 renamed CRAM_PLANS -> CRAM_COMMON


### [0.2.2] - 2016-11-15

2016-02-22 [robot_interfaces] added ROBOT-ODOM-FRAME predicate.

2016-02-22 [cram_tf] moved dynamic variable initialization here from belief state

2016-02-22 [cram-tf] a bit more failure handling for ROBOT-CURRENT-POSE.

2016-02-22 CRAM_TRANSFORMS_STAMPED -> CRAM_TF

2016-02-22 [cram_tf] fixed a bug in INIT-TF: use MACROLET instead of FLET.

2016-03-04 [cram_tf] moved pose-related facts from BTR here

2016-03-04 [cm] *Z-PADDING* should be a parameter, not DEFVAR.

2016-05-24 [failures] added new types of failures

2016-07-15 [plan-lib] Changed `perceive-object` modifiers from symbols into keywords

2016-07-18 [plan-lib] don't export perceive-object quantifiers: they're keywords now

2016-07-19 [cram-tf] slight improvements for cram-tf initialization: changed default timeout in simulation to 10s, otherwise to 4s, also nicified the debug statements.

2016-07-21 [cram-plan-library] used the new matching-avail-pm function

2016-07-21 [plan-lib] added functionality to perform activities

2016-07-27 [robot-interfaces] added robot-tool-frame predicate: for storing the tool frames of the arms

2016-08-11 [robot-interfaces] Added predicates to get arm links and joints, hand links

2016-08-25 [robot-interfaces] Predicates to read joint properties from the urdf, and to describe groups of links/joints.

2016-11-14 [gauss_costmap] commented out BULLET-GENERATOR: it wasn't used anywhere anyway and allowed to get rid of bullet dependency

2016-11-15 cleaned up package.xmls


### [0.2.1] - 2016-01-28
