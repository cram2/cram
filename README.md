### STILL WIP!! NOT USEABLE YET


![Screenshot](https://raw.githubusercontent.com/cram2/cram/master/graphics/CramLogoSmall.png)
=============

### What is CRAM

CRAM is a toolbox for designing, implementing and deploying software on autonomous robots. The framework provides various tools and libraries for aiding in robot software development as well as geometric reasoning and fast simulation mechanisms to develop cognition-enabled control programs that achieve high lev- els of robot autonomy. CRAM also provides tools for introspection that enable the robots to reason about their past executions and improve by autonomously optimizing their plans.

The core packages of CRAM are implemented in Common Lisp (with a little bit of C/C++) with support to the ROS middleware infrastructure.

### Installation

*  If you wish to install cram please follow the instruction here: [cram-installation guide](http://cram-system.org/installation).


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


### Getting started

*  [cram-getting-started](http://cram-system.org/doc/getting_started).


### Workspace Installation
  * `sudo apt-get install python3-rosinstall python3-wstool`
  * `wget https://raw.githubusercontent.com/cram2/cram/boxy-noetic/cram-install.sh`
  * `chmod +x cram-install.sh`
  * `./cram-install.sh`
  * OPTIONAL: `echo source ~/roscram/cram_ws/devel/setup.bash" >> ~/.bashrc`  For sourcing automaticly
  * OPTIONAL: `echo "alias repl='rosrun roslisp_repl roslisp_repl'" >> ~/.bashrc`  For starting the repl with `repl` and not `rosrun....`


For ROS noetic and the current packages, one thing needs to be fixed. The package octomap contains a dependency to a ROS2 package, which can be ignored. Open the package.xml of octomap.

* `roscd octomap`
* `sudo nano package.xml`
Now remove the following line within:

* `<exec_depend condition="$ROS_VERSION == 2">ament_cmake</exec_depend>`
