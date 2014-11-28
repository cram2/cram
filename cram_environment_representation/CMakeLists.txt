cmake_minimum_required(VERSION 2.8.3)
project(cram_environment_representation)

find_package(catkin REQUIRED COMPONENTS
  cram_reasoning
  cram_utilities
  cram_designators
  cram_roslisp_common
  cram_projection
  bullet_reasoning
  cram_plan_knowledge
  cram_plan_library
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

