cmake_minimum_required(VERSION 2.8.3)
project(cram_execution_trace_server)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  cram_prolog
  cram_plan_occasions_events
  cram_execution_trace
  cram_plan_library
  roslisp_utilities
  cram_utilities
  cram_json_prolog
)

add_service_files(
  FILES
  ListExecutionTraces.srv
  SelectExecutionTrace.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  cram_prolog
  cram_plan_occasions_events
  cram_execution_trace
  cram_plan_library
  roslisp_utilities
  cram_utilities
  cram_json_prolog
)