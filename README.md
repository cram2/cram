# CRAM 

![CRAM Logo](https://raw.githubusercontent.com/cram2/cram/master/graphics/CramLogoSmall.png)

## Introduction
**CRAM** is a versatile toolbox tailored for the design, implementation, and deployment of software on autonomous robots. The framework offers:

- Tools and libraries to streamline robot software development.
- Geometric reasoning and rapid simulation capabilities for crafting cognition-enabled control programs, heightening robot autonomy.
- Introspective tools, allowing robots to reflect on past actions, facilitating autonomous plan optimization.

The foundational packages of CRAM are crafted in Common Lisp (supplemented with C/C++), and are fully integrated with the ROS middleware infrastructure.

## Important Note
To utilize CRAM with a real robot or in tandem with Unreal Engine/Giskard, the `cram_integration` repository is essential.

## Version Information
The current branch is tested with **20.04**. As it's continually evolving due to ongoing development, any discovered bugs are encouraged to be reported as an issue.

## Prerequisites

- Complete the [ROS installation](http://wiki.ros.org/noetic/Installation).
- Set up an [SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh) for your GitHub account.

## Workspace Setup

1. Install required ROS packages:
   ```bash
   sudo apt install ros-noetic-roslisp-repl
   sudo apt-get install python3-rosinstall python3-wstool
   ```

2. Navigate to your workspace's `src` directory and initialize it:
   ```bash
   cd ~/workspace/src
   wstool init
   ```

3. Merge and update the workspace:
   ```bash
   wstool merge https://raw.githubusercontent.com/cram2/cram/devel/cram-20.04.rosinstall
   wstool update
   ```

4. Return to the workspace root, update ROS dependencies, and build the workspace:
   ```bash
   cd ~/workspace/
   rosdep update
   rosdep install --ignore-src --from-paths src/ -r
   catkin_make
   ```

5. For ROS Noetic users, a dependency adjustment is needed for the octomap package:
   ```bash
   roscd octomap
   sudo nano package.xml
   ```
   Once inside the XML, omit the line:
   ```xml
   <exec_depend condition="$ROS_VERSION == 2">ament_cmake</exec_depend>
   ```

## Testing & Continuous Integration

When proposing changes via a **Pull Request**:

- Ensure the CI successfully compiles CRAM.
- Ascertain the milestone-demo's functionality.
- Confirm the operational status of tests:
   ```bash
   roslaunch cram_integration_tests pr2.launch 
   ```
   In a subsequent terminal:
   ```lisp
   (swank:operate-on-system-for-emacs "cram-integration-tests-pr2" (quote load-op))
   (roslisp-utilities:startup-ros)
   (in-package cram-integration-tests)
   (lisp-unit:run-tests)
   ```
   The outcome should resemble [this reference image](https://github.com/cram2/cram/blob/noetic/graphics/test-summary.png).

## Documentation
For more detailed information, we've also initiated a wiki on GitHub. Explore it [here](https://github.com/cram2/cram/wiki).

## Change Log: Major Restructure Details

- 31/07-09:30: Transferred components to external interface.
- 31/07-10:30: Introduced robots to `cram_robots`.
- 31/07-10:50: Resolved issues with Tiago in cram_robots.
- 31/07-11:10: Transitioned robots into the external section.
- 31/07-12:44: Renamed cram_external_interfaces to cram_integration.
- 31/07-13:41: Integrated actions into cram_actions.
- 31/07-13:50: Incorporated robot_description into cram_integration.
- 31/07-14:18: Relocated *costmaps to cram_locations.
- 31/07-14:30: Renamed cram_common to cram_interface.
- 31/07-13:34: Transitioned cram_3d_world to cram_bullet_world.
- 07/08-2:13: Shifted files out of external_interfaces.
- 07/08-2:43: Relocated files to bullet world.
