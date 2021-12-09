#!/bin/sh
# Symlink for config files
install -d ~/roscram/cram_ws/src
cd ~/roscram/cram_ws/src
wstool init
wstool merge https://raw.githubusercontent.com/cram2/cram/noetic/cram-20.04.rosinstall
wstool update
cd ..
rosdep update
rosdep install --ignore-src --from-paths src/ -r -y
install -d ~/roscram/ros_emacs_utils_ws/src
cd ~/roscram/ros_emacs_utils_ws/src
git clone git@github.com:code-iai/ros_emacs_utils.git
cd ..
#install is necessary
catkin_make install
catkin_make
source devel/setup.bash
cd ~/roscram/cram_ws
catkin config --extend ~/roscram/ros_emacs_utils_ws/devel
catkin build

