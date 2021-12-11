### STILL WIP!! NOT USEABLE YET


![Screenshot](https://raw.githubusercontent.com/cram2/cram/master/graphics/CramLogoSmall.png)
=============

### What is CRAM

CRAM is a toolbox for designing, implementing and deploying software on autonomous robots. The framework provides various tools and libraries for aiding in robot software development as well as geometric reasoning and fast simulation mechanisms to develop cognition-enabled control programs that achieve high lev- els of robot autonomy. CRAM also provides tools for introspection that enable the robots to reason about their past executions and improve by autonomously optimizing their plans.

The core packages of CRAM are implemented in Common Lisp (with a little bit of C/C++) with support to the ROS middleware infrastructure.

----
### Workspace Installation
Note: The script *cram-install.sh* is made available because the apt-get pkg has not yet been updated, but this will happen in the future. In the installation script, you can see that the *ros_emacs_utils* pkg is built in a separate *catkin_workspace*, this is due the catkin install command, which is important for roslisp_repl and slime.
  * `sudo apt-get install python3-rosinstall python3-wstool python3-catkin-tools`
  * `https://raw.githubusercontent.com/cram2/cram/popcorn-noetic/cram-install.sh
  * `chmod +x cram-install.sh`
  * `./cram-install.sh`
  * OPTIONAL: `echo "source ~/roscram/cram_ws/devel/setup.bash" >> ~/.bashrc`  For sourcing automaticly
  * OPTIONAL: `echo "alias repl='rosrun roslisp_repl roslisp_repl'" >> ~/.bashrc`  For starting the repl with `repl` otherwise you have to start it with `rosrun roslisp_repl roslisp_repl`


For ROS noetic and the current packages, one thing needs to be fixed. The package octomap contains a dependency to a ROS2 package, which can be ignored. Open the package.xml of octomap.

* `roscd octomap`
* `sudo nano package.xml`
Now remove the following line within:

* `<exec_depend condition="$ROS_VERSION == 2">ament_cmake</exec_depend>`

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
  * Popcorndemo: [Working]
    * To use this please checkout in iai_maps the **popcorn** branch. 

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

