cmake_minimum_required(VERSION 2.8.3)
project(bullet_reasoning)

find_package(catkin REQUIRED COMPONENTS
  alexandria
  cram_prolog
  cram_manipulation_knowledge
  cl_bullet
  cl_bullet_vis
  cram_json_prolog
  cl_urdf
  cl_tf
  cram_semantic_map_utils
  cram_designators
  location_costmap
  roslisp_utilities
  cram_roslisp_common
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

