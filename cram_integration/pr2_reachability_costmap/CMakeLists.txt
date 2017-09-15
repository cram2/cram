cmake_minimum_required(VERSION 2.8.3)
project(pr2_reachability_costmap)

find_package(catkin REQUIRED COMPONENTS
  cram_language
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_lisp_executable(
#  bin/generate_reachability_map pr2-reachability-costmap
#  pr2-reachability-costmap:generate-map-main)
