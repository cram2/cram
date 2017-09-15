cmake_minimum_required(VERSION 2.8.3)
project(cram_bullet_reasoning)

find_package(catkin REQUIRED COMPONENTS
  alexandria
  cram_prolog
  cram_robot_interfaces
  cl_bullet
  cl_bullet_vis
  cl_urdf
  cl_tf
  cram_semantic_map_utils
  cram_designators
  cram_location_costmap
  roslisp_utilities
  cram_utilities
  cram_tf
  cl_transforms_stamped
  cl_transforms
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

