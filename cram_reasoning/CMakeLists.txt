cmake_minimum_required(VERSION 2.8.3)
project(cram_reasoning)

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

install (FILES cram-reasoning.asd cram-reasoning-tests.asd
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
