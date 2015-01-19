cmake_minimum_required(VERSION 2.8.3)
project(pr2_navigation_process_module)

find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  actionlib_lisp
  cram_process_modules
  cram_designators
  cram_reasoning
  roslisp_utilities
  cram_roslisp_common
  cram_plan_failures
  cram_plan_knowledge
  cram_moveit
)

catkin_package()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


