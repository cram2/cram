![Screenshot](https://raw.githubusercontent.com/cram2/cram/master/graphics/CramLogoSmall.png)
=============

### What is CRAM

CRAM is a toolbox for designing, implementing and deploying software on autonomous robots. The framework provides various tools and libraries for aiding in robot software development as well as geometric reasoning and fast simulation mechanisms to develop cognition-enabled control programs that achieve high lev- els of robot autonomy. CRAM also provides tools for introspection that enable the robots to reason about their past executions and improve by autonomously optimizing their plans.

The core packages of CRAM are implemented in Common Lisp (with a little bit of C/C++) with support to the ROS middleware infrastructure.

----
### Version
The branch is currently tested on **20.04**. It is under constant change due the development. If you notice a bug, please report it in an issue.

### Prerequisite

[ROS installation](http://wiki.ros.org/noetic/Installation) and an [SSH key set up](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh)  for your github account.

### Workspace Installation
* `sudo apt install ros-noetic-roslisp-repl`
* `sudo apt-get install python3-rosinstall python3-wstool`
* `cd ~/workspace/src`
* `wstool init`
* `wstool merge https://raw.githubusercontent.com/cram2/cram/devel/cram-20.04.rosinstall`
* `wstool update`
* `cd ~/workspace/`
* `rosdep update`
* `rosdep install --ignore-src --from-paths src/ -r`
* `catkin_make`


For ROS noetic and the current packages, one thing needs to be fixed. The package octomap contains a dependency to a ROS2 package, which can be ignored. Open the package.xml of octomap.

* `roscd octomap`
* `sudo nano package.xml`
Now remove the following line within:

* `<exec_depend condition="$ROS_VERSION == 2">ament_cmake</exec_depend>`

----
### Testing & Continuous Integration
If changes are made to the code and a **PullRequest** results from it, please check the following things:
* The automatic CI, has to build *CRAM* succesfully.
* Is the milestone-demo still running correctly?
* Are the test still running? 
   * New Terminal: roslaunch cram_integration_tests pr2.launch 
   * `(swank:operate-on-system-for-emacs "cram-integration-tests-pr2" (quote load-op))`
   * `(roslisp-utilities:startup-ros)`
   * `(in-package cram-integration-tests`
   * `(lisp-unit:run-tests)`
   * The result should look like this: [Test-Summary-IMG](https://github.com/cram2/cram/blob/noetic/graphics/test-summary.png)

----
### Current State of Demos:
* PR2
  * Unit-test: all-test [Working] 
  * Milestone: setting-demo [Working]
  * Milestone: cleaning-demo [Working]
 
----

### Directory
* **cram_3d_world** Bullet physics engine-based and OpenGl offscreen rendering-based reasoning mechanisms.
* **cram_3rdparty** 3rd party Lisp liabraries wrapped into ROS packages.
* **cram_boxy** Hardware interface for using Boxy robot in CRAM and lightweight simulation.
* **cram_common** Common libraries building on top of CRAM core. Including costmap and implementations.
* **cram_core** Tools and Interfaces for writing Cognition-Enabled Reactive Concurrent Plans.
* **cram_json_prolog** ROS JSON Prolog client implementation in Lisp: sending Prolog queries in JSON format over ROS. 
* **cram_knowrob** Libraries for using knowrob in Lisp.
* **cram_pr2** CRAM packages related to the PR2, e.g. process modules, costmaps and lightweight simulation of PR2 robot. Also, fetch and deliver plans for PR2 robot.
* **cram_roboserlock** Interface for communicating with RoboSherlock from CRAM.
* **cram_tutorials** Tutorials for: [cram-turtorials](http://cram-system.org/tutorials).

