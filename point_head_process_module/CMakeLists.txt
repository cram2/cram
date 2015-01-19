cmake_minimum_required(VERSION 2.8.3)
project(point_head_process_module)

find_package(catkin REQUIRED COMPONENTS
  pr2_controllers_msgs
  actionlib_lisp
  cram_process_modules
  cram_designators
  cram_reasoning
  cram_roslisp_common
  cram_plan_knowledge
  roslisp_utilities
  alexandria
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

