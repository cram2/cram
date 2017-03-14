cmake_minimum_required(VERSION 2.8.3)
project(cram_commander)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_service_files(FILES
  ReferenceDesignator.srv
  PerformDesignator.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)