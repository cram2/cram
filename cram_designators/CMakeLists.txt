cmake_minimum_required(VERSION 2.8.3)
project(cram_designators)

find_package(catkin REQUIRED) 

catkin_package()

foreach(dir
    src
    test)
  install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
      PATTERN ".svn" EXCLUDE
      PATTERN ".git" EXCLUDE)
endforeach()

install (FILES designators.asd designators-test.asd
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
