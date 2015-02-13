cmake_minimum_required(VERSION 2.8.3)
project(map_annotation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  cram_reasoning
  cram_roslisp_common
  cl_transforms
  message_generation
)

add_service_files(
  FILES
  FindClosestAnnotation.srv
  GetAnnotatedPoints.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  rospy
  cram_reasoning
  cram_roslisp_common
  cl_transforms
  message_runtime
)