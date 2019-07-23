cmake_minimum_required(VERSION 2.8.3)
project(cram_plan_transformation)

find_package(catkin REQUIRED COMPONENTS
  alexandria
  cram_language
  roslisp
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
