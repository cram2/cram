cmake_minimum_required(VERSION 2.8.3)
project(cram_plan_actionserver)
find_package(catkin REQUIRED actionlib_msgs message_generation)

add_action_files(
  FILES
  ExecutePlan.action
)

add_service_files(
  FILES
  PlanList.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)
