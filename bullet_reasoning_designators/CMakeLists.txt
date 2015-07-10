cmake_minimum_required(VERSION 2.8.3)
project(bullet_reasoning_designators)

find_package(catkin REQUIRED COMPONENTS
  cram_prolog
  designators_ros
  cram_utilities
  bullet_reasoning
  cram_designators
  location_costmap
  cram_environment_representation
  cram_plan_knowledge
  roslisp_utilities
  alexandria
  cram_moveit
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

