cmake_minimum_required(VERSION 2.8.3)
project(cram_physics_utils)
find_package(catkin REQUIRED)
catkin_package()

# assimp headers have different names between different Ubuntu versions.
# We patch the src/assimp-grovel.lisp file when necessary to adjust to that.
find_file(HAVE_CIMPORT_H assimp/cimport.h)
if(NOT HAVE_CIMPORT_H)
  find_file(HAVE_ASSIMP_H assimp/assimp.h)
  if(HAVE_ASSIMP_H)
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/patch/backup)
      execute_process(COMMAND patch -d ${CMAKE_CURRENT_SOURCE_DIR} -p0 -N -r -
        -b -B patch/backup/
        -i patch/assimp-grovel-12.04.patch)
      execute_process(COMMAND git config filter.gitignore.clean
        "patch -d ${CMAKE_CURRENT_SOURCE_DIR} -p0 -N -r - -R -s -i patch/assimp-grovel-12.04.patch -o -"
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
      execute_process(COMMAND git config filter.gitignore.smudge cat
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
      execute_process(COMMAND git add src/assimp-grovel.lisp
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
  else()
    message(SEND_ERROR "Can't find assimp headers.")
  endif()
endif()