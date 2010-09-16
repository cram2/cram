cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Release)

rosbuild_init()

rospack_add_lisp_executable(bin/drive_to_waypoints kipla kipla:drive-to-waypoints-main)
rospack_add_lisp_executable(bin/kipla_action kipla cram-actionserver:server)
