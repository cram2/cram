cmake_minimum_required(VERSION 2.8.3)
project(map_annotation)
find_package(catkin REQUIRED geometry_msgs)

add_service_files(
  FILES
  FindClosestAnnotation.srv
  GetAnnotatedPoints.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)
