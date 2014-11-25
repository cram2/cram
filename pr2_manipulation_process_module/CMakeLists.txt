cmake_minimum_required(VERSION 2.8.3)
project(pr2_manipulation_process_module)

find_package(catkin REQUIRED COMPONENTS
  cram_language
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

