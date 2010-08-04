cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Release)

rosbuild_init()

rospack_add_lisp_executable(bin/demo-counter-to-table kipla kipla:run-demo-counter-to-table)
rospack_add_lisp_executable(bin/demo-table-to-counter kipla kipla:run-demo-table-to-counter)
rospack_add_lisp_executable(bin/drive_to_waypoints kipla kipla:drive-to-waypoints-main)
