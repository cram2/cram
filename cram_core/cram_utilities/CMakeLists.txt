cmake_minimum_required(VERSION 3.0.2)
project(cram_utilities)

find_package(catkin REQUIRED) 

catkin_package()

foreach(dir
    src
    tests)
  install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
      PATTERN ".svn" EXCLUDE
      PATTERN ".git" EXCLUDE)
endforeach()

install (FILES cram-utilities.asd cram-utilities-tests.asd
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
