cmake_minimum_required(VERSION 2.8.3)
project(cl_bullet)

find_package(catkin REQUIRED COMPONENTS
  cffi
  cffi_ros_utils
)
find_package(PkgConfig REQUIRED)

pkg_check_modules(BULLET bullet)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package(
  LIBRARIES ${CATKIN_PACKAGE_BIN_DESTINATION}/bullet_cl
)

add_library(bullet_cl
  src/dynamics_world.cpp src/rigid_body.cpp
  src/motion_state.cpp src/collision_shapes.cpp
  src/constraints.cpp src/debug_draw.cpp)

install(TARGETS bullet_cl
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (BULLET_FOUND)
   include_directories(${BULLET_INCLUDE_DIRS})
   target_link_libraries(bullet_cl ${BULLET_LIBRARIES})
endif (BULLET_FOUND)

