cmake_minimum_required(VERSION 2.8.3)
project(cl_urdf)

find_package(catkin REQUIRED COMPONENTS
  roslisp
  cl_transforms
  cram_physics_utils
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package()

