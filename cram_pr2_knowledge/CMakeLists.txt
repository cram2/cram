cmake_minimum_required(VERSION 2.8.3)
project(cram_pr2_knowledge)

find_package(catkin REQUIRED COMPONENTS
  cram_prolog
  cram_robot_interfaces
  bullet_reasoning
  cram_designators
  cl_transforms_stamped
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

