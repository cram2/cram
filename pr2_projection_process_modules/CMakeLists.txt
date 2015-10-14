cmake_minimum_required(VERSION 2.8.3)
project(pr2_projection_process_modules)

find_package(catkin REQUIRED COMPONENTS
  alexandria
  cram_plan_failures
  cram_process_modules
  cram_bullet_reasoning
  cram_bullet_reasoning_designators
  cram_designators
  cram_plan_occasions_events
  cram_occasions_events
  cram_bullet_reasoning_belief_state
  cram_robot_interfaces
  cl_transforms
  cram_projection
  cram_pr2_designators
  cram_semantic_map_utils
  cram_semantic_map
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

