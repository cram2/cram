cmake_minimum_required(VERSION 3.0.2)
project(cram_json_prolog)

find_package(catkin REQUIRED COMPONENTS
  cram_language
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

