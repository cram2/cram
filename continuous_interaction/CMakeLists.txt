cmake_minimum_required(VERSION 2.8.3)
project(continuous_interaction)

find_package(catkin REQUIRED COMPONENTS alexandria cram_language cram_plan_knowledge cram_plan_library cram_reasoning cram_roslisp_common designators_ros object_location_designators roslisp)

catkin_package(
  CATKIN_DEPENDS alexandria cram_language cram_plan_knowledge cram_plan_library cram_reasoning cram_roslisp_common designators_ros object_location_designators roslisp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
