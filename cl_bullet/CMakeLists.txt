cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

find_package(Bullet)

rosbuild_add_library(bullet_cl
  src/dynamics_world.cpp src/rigid_body.cpp
  src/motion_state.cpp src/collision_shapes.cpp
  src/constraints.cpp src/debug_draw.cpp)

if (BULLET_FOUND)
   include_directories(${BULLET_INCLUDE_DIRS})
   target_link_libraries(bullet_cl ${BULLET_LIBRARIES})
endif (BULLET_FOUND)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
