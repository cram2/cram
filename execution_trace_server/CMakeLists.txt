cmake_minimum_required(VERSION 2.8.3)
project(execution_trace_server)
find_package(catkin REQUIRED message_generation)

add_service_files(
  FILES
  ListExecutionTraces.srv
  SelectExecutionTrace.srv
)

generate_messages()

catkin_package(
  DEPENDS
  message_runtime
)