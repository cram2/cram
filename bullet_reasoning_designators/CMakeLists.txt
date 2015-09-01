cmake_minimum_required(VERSION 2.8.3)
project(bullet_reasoning_designators)

find_package(catkin REQUIRED COMPONENTS
  cram_prolog
  cram_transforms_stamped
  cram_utilities
  bullet_reasoning
  cram_designators
  cram_location_costmap
  cram_environment_representation
  cram_manipulation_knowledge
  roslisp_utilities
  alexandria
  cram_moveit
  cl_opengl
  cl_bullet
  cl_bullet_vis
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

