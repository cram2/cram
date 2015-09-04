cmake_minimum_required(VERSION 2.8.3)
project(cram_bullet_reasoning_belief_state)

find_package(catkin REQUIRED COMPONENTS
  cram_prolog
  cram_utilities
  cram_designators
  cram_roslisp_common
  cram_projection
  cram_bullet_reasoning
  cram_plan_occasions_events
  cram_occasions_events
  cram_plan_library
  cram_physics_utils
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

